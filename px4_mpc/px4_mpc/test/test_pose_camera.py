import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mpc_msgs.srv import SetPose
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import px4_mpc.utils.ros_utils as ros_utils
from tf2_geometry_msgs import do_transform_pose
import tf2_py
import tf_transformations as tft
import numpy as np


class VisualServo(Node):
    def __init__(self):
        super().__init__("visual_servo")

        self.namespace = self.declare_parameter('namespace', '').value
        self.namespace_prefix = f'/{self.namespace}' if self.namespace else ''

        # Create a subscription to the pose topic
        self.object_pose_sub = self.create_subscription(
            PoseStamped, "/pose/icp_result", self.pose_callback, 10
        )

        self.goal_pose_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)


        self.goal_pose_pub = self.create_publisher(PoseStamped, f'{self.namespace_prefix}/px4_mpc/setpoint_pose', 10)
        # Create a client for the set_pose service
        self.client = self.create_client(SetPose, "/set_pose")

        # Wait for service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service /set_pose not available, waiting...")

        # Add pose history buffer for consistency checking
        self.pose_history = []
        self.history_size = 5
        self.position_threshold = 0.05

        # State variable for docking
        self.run_docking = False
        self.x_offset = 0.8

        # Add TF buffer and listener for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create service for docking control
        self.srv = self.create_service(
            SetBool, "run_docking", self.run_docking_callback
        )

        self.get_logger().info(
            "Pose forwarder initialized. Use run_docking service to enable/disable."
        )

        # spawn pose #1
        self.init_pos = np.array(
            [-0.01404785, 1.36248684, 0.0]
        )  # inverted z and y axis
        self.init_att = np.array(
            [8.92433882e-01, -5.40154197e-08, 4.97020096e-08, -4.51177984e-01]
        )  # invered z and y axis

        self.move_robot(self.init_pos, self.init_att)

    def move_robot(self, position, orientation):
        init_poseStamped = PoseStamped()
        init_poseStamped.header.frame_id = "map"
        init_poseStamped.header.stamp = self.get_clock().now().to_msg()
        init_poseStamped.pose.position.x = position[0]
        init_poseStamped.pose.position.y = position[1]
        init_poseStamped.pose.position.z = position[2]
        init_poseStamped.pose.orientation.w = orientation[0]
        init_poseStamped.pose.orientation.x = orientation[1]
        init_poseStamped.pose.orientation.y = orientation[2]
        init_poseStamped.pose.orientation.z = orientation[3]

        self.goal_pose_pub.publish(init_poseStamped)
        request = SetPose.Request()
        request.pose = init_poseStamped.pose

        # Send the request asynchronously
        future = self.client.call_async(request)
        future.add_done_callback(self.service_callback)

    def run_docking_callback(self, request, response):
        """Service callback to enable/disable pose forwarding"""
        self.run_docking = request.data
        response.success = True
        if self.run_docking:
            response.message = "Docking mode enabled"
            self.get_logger().info("Docking mode enabled")
        else:
            response.message = "Docking mode disabled"
            self.get_logger().info("Docking mode disabled")
        return response

    def transform_pose(self, pose_stamped):
        """
        Transform pose from camera frame to map frame and apply offset + rotation.
        """
        try:
            if not self.tf_buffer.can_transform(
                pose_stamped.header.frame_id, "map", rclpy.time.Time()
            ):
                return None
            else:
                # First translate the pose_stamped by x axis in camera frame
                # First transform the pose from camera_link to map frame
                transform = self.tf_buffer.lookup_transform(
                    "map",  # target frame
                    pose_stamped.header.frame_id,  # source frame
                    rclpy.time.Time(),  # get the latest transform
                    rclpy.duration.Duration(seconds=1.0),  # timeout
                )

                # Apply the transform to get pose in map frame
                map_pose = do_transform_pose(pose_stamped.pose, transform)

                # Now apply the offset and rotation in the map frame
                transformed_pose = map_pose

                # Add offset along x axis (in map frame)
                transformed_pose.position.y += self._offset

                # Apply 180-degree rotation around z in map frame
                # (simplified approach using quaternion components)
                transformed_pose.orientation.x = -transformed_pose.orientation.x
                transformed_pose.orientation.y = -transformed_pose.orientation.y

                # Create a new PoseStamped for the result
                result_pose = PoseStamped()
                result_pose.header.frame_id = "map"
                result_pose.header.stamp = self.get_clock().now().to_msg()
                result_pose.pose = transformed_pose

                self.get_logger().info(
                    f"Transformed pose to map frame - Position: ({transformed_pose.position.x:.2f}, "
                    f"{transformed_pose.position.y:.2f}, {transformed_pose.position.z:.2f})"
                )

                return result_pose

        except Exception as e:
            self.get_logger().error(f"Transform error: {e}")
            return None

    def pose_callback(self, msg):
        # transformed_pose_stamped = None
        # Store the pose in history for consistency check
        self.pose_history.append(msg.pose)

        # Keep only the most recent poses
        if len(self.pose_history) > self.history_size:
            self.pose_history.pop(0)

        # Only forward the pose if it's consistent and run_docking is True
        if self.run_docking and self.is_pose_consistent():
            self.run_docking = False
            self.get_logger().info("Pose consistent, transforming to map frame")

            T_cam_obj = ros_utils.pose_to_matrix(msg.pose)
            # Transform the pose to map frame and apply offset/rotation
            T_cam_goal = ros_utils.offset_in_front(T_cam_obj, self.x_offset)

            # publish t_cam_goal into poseStamped message
            # q = tft.quaternion_from_matrix(T_cam_goal)              # (x,y,z,w)
            # p = T_cam_goal[:3, 3]
            # transformed_pose_stamped = PoseStamped()
            # transformed_pose_stamped.header.frame_id = "base_link"
            # transformed_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            # transformed_pose_stamped.pose.position.x = p[0]
            # transformed_pose_stamped.pose.position.y = p[1]
            # transformed_pose_stamped.pose.position.z = p[2]
            # transformed_pose_stamped.pose.orientation.x = q[0]
            # transformed_pose_stamped.pose.orientation.y = q[1]
            # transformed_pose_stamped.pose.orientation.z = q[2]
            # transformed_pose_stamped.pose.orientation.w = q[3]

            try:
                if not self.tf_buffer.can_transform(
                    msg.header.frame_id, "map", rclpy.time.Time()
                ):

                    transformed_pose_stamped = None
                else:
                    transform = self.tf_buffer.lookup_transform(
                        "map",  # target frame
                        msg.header.frame_id,  # source frame
                        rclpy.time.Time(),  # get the latest transform
                        rclpy.duration.Duration(seconds=1.0),  # timeout
                    )

                    transformed_pose_stamped = ros_utils.matrix_to_posestamped(
                        T_cam_goal, transform, "map", self.get_clock().now().to_msg()
                    )
            except Exception as e:
                self.get_logger().error(f"Transform error: {e}")
                transformed_pose_stamped = None

            if transformed_pose_stamped is not None:
                self.goal_pose_pub.publish(transformed_pose_stamped)
                request = SetPose.Request()
                request.pose = transformed_pose_stamped.pose

                # Send the request asynchronously
                future = self.client.call_async(request)
                future.add_done_callback(self.service_callback)
            else:
                self.get_logger().error("Failed to transform pose, not sending request")
        elif not self.run_docking:
            self.get_logger().debug("run_docking is False, not forwarding pose")
        elif not self.is_pose_consistent():
            self.get_logger().debug("Pose not consistent yet, waiting for stability")

    def is_pose_consistent(self):
        """Check if the recent poses are consistent within threshold"""
        if len(self.pose_history) < self.history_size:
            return False

        # Check the last self.history_size poses
        reference_pose = self.pose_history[-1]
        for i in range(-2, -self.history_size - 1, -1):
            pose = self.pose_history[i]

            # Calculate Euclidean distance between positions
            dx = reference_pose.position.x - pose.position.x
            dy = reference_pose.position.y - pose.position.y
            dz = reference_pose.position.z - pose.position.z
            distance = (dx**2 + dy**2 + dz**2) ** 0.5

            if distance > self.position_threshold:
                print(
                    f"Pose inconsistency detected: {distance:.2f} > {self.position_threshold:.2f}"
                )

                return False

        return True

    def service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service call succeeded. Response: {response}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main():
    rclpy.init()
    node = VisualServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Pose forwarder stopped by user")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
