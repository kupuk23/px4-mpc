import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mpc_msgs.srv import SetPose
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

from tf2_geometry_msgs import do_transform_pose
import tf2_py

class PoseForwarder(Node):
    def __init__(self):
        super().__init__('pose_forwarder')
        
        # Create a subscription to the pose topic
        self.object_pose_sub = self.create_subscription(
            PoseStamped,
            '/pose/icp_result',
            self.pose_callback,
            10)
        
        self.goal_pose_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10)

        # Create a client for the set_pose service
        self.client = self.create_client(SetPose, '/set_pose')  
        
        # Wait for service to become available
        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service /set_pose not available, waiting...')

        # Add pose history buffer for consistency checking
        self.pose_history = []
        self.history_size = 5
        self.position_threshold = 0.05
        
        # State variable for docking
        self.run_docking = False
        self.y_offset = 0.7
        
        # Add TF buffer and listener for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create service for docking control
        self.srv = self.create_service(
            SetBool,
            'run_docking',
            self.run_docking_callback)
        
        self.get_logger().info('Pose forwarder initialized. Use run_docking service to enable/disable.')


    def run_docking_callback(self, request, response):
        """Service callback to enable/disable pose forwarding"""
        self.run_docking = request.data
        response.success = True
        if self.run_docking:
            response.message = "Docking mode enabled"
            self.get_logger().info('Docking mode enabled')
        else:
            response.message = "Docking mode disabled"
            self.get_logger().info('Docking mode disabled')
        return response
    
    def transform_pose(self, pose_stamped):
        """
        Transform pose from camera frame to map frame and apply offset + rotation.
        """
        try:
            if not self.tf_buffer.can_transform(
                    pose_stamped.header.frame_id, 'map', rclpy.time.Time()
                ):
                return None
            else:
                # First transform the pose from camera_link to map frame
                transform = self.tf_buffer.lookup_transform(
                    'map',                # target frame
                    pose_stamped.header.frame_id,  # source frame
                    rclpy.time.Time(),    # get the latest transform
                    rclpy.duration.Duration(seconds=1.0)  # timeout
                )
                
                # Apply the transform to get pose in map frame
                map_pose = do_transform_pose(pose_stamped.pose, transform)
                
                # Now apply the offset and rotation in the map frame
                transformed_pose = map_pose
                
                # Add offset along x axis (in map frame)
                transformed_pose.position.y += self.y_offset
                
                # Apply 180-degree rotation around z in map frame 
                # (simplified approach using quaternion components)
                transformed_pose.orientation.x = -transformed_pose.orientation.x
                transformed_pose.orientation.y = -transformed_pose.orientation.y
                
                # Create a new PoseStamped for the result
                result_pose = PoseStamped()
                result_pose.header.frame_id = 'map'
                result_pose.header.stamp = self.get_clock().now().to_msg()
                result_pose.pose = transformed_pose
                
                self.get_logger().info(f'Transformed pose to map frame - Position: ({transformed_pose.position.x:.2f}, '
                                f'{transformed_pose.position.y:.2f}, {transformed_pose.position.z:.2f})')
                
                return result_pose
            
        except Exception as e:
            self.get_logger().error(f'Transform error: {e}')
            return None

    def pose_callback(self, msg):
        # Store the pose in history for consistency check
        self.pose_history.append(msg.pose)
        
        # Keep only the most recent poses
        if len(self.pose_history) > self.history_size:
            self.pose_history.pop(0)
        
        # Only forward the pose if it's consistent and run_docking is True
        if self.run_docking and self.is_pose_consistent():
            self.run_docking = False
            self.get_logger().info('Pose consistent, transforming to map frame')
            
            # Transform the pose to map frame and apply offset/rotation
            transformed_pose_stamped = self.transform_pose(msg)
            
            
            
            if transformed_pose_stamped is not None:
                self.goal_pose_pub.publish(transformed_pose_stamped)
                request = SetPose.Request()
                request.pose = transformed_pose_stamped.pose
                
                # Send the request asynchronously
                # future = self.client.call_async(request)
                # future.add_done_callback(self.service_callback)
            else:
                self.get_logger().error('Failed to transform pose, not sending request')
        elif not self.run_docking:
            self.get_logger().debug('run_docking is False, not forwarding pose')
        elif not self.is_pose_consistent():
            self.get_logger().debug('Pose not consistent yet, waiting for stability')

    def is_pose_consistent(self):
        """Check if the recent poses are consistent within threshold"""
        if len(self.pose_history) < self.history_size:
            return False
        
        # Check the last self.history_size poses
        reference_pose = self.pose_history[-1]
        for i in range(-2, -self.history_size-1, -1):
            pose = self.pose_history[i]
            
            # Calculate Euclidean distance between positions
            dx = reference_pose.position.x - pose.position.x
            dy = reference_pose.position.y - pose.position.y
            dz = reference_pose.position.z - pose.position.z
            distance = (dx**2 + dy**2 + dz**2)**0.5
            
            if distance > self.position_threshold:
                print(f'Pose inconsistency detected: {distance:.2f} > {self.position_threshold:.2f}')
                
                return False
        
        return True
    
    def service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Service call succeeded')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    

def main():
    rclpy.init()
    node = PoseForwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Pose forwarder stopped by user')
        rclpy.shutdown()

if __name__ == '__main__':
    main()