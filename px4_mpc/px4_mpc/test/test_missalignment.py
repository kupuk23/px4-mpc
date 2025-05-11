import numpy as np
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


def q_to_rot_mat(q):
    """Body‑to‑inertial rotation from unit quaternion (same formula as in your model)."""
    qw, qx, qy, qz = q
    return np.array(
        [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
            [2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)],
            [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)],
        ]
    )


def misalignment_angle(p_robot_I, q_robot, p_obj_I):
    """
    Returns camera–object angle θ  [rad].
    Camera optical axis = +x_B  = [1,0,0] in body frame.
    """
    deg = 180 / np.pi
    c_B = np.array([1.0, 0.0, 0.0])  # camera axis in body
    
    # 2D CASE #
    p_obj_I[2] = 0.0  # set z to 0 since we are in 2D
    p_robot_I[2] = 0.0  # set z to 0 since we are in 2D

    r_I = p_obj_I - p_robot_I  # object bearing in inertial
    r_b = q_to_rot_mat(q_robot).T @ r_I  # inertial → body
    cos_theta = np.dot(c_B, r_b) / np.linalg.norm(r_b)
    theta = np.arccos(np.clip(cos_theta, -1.0, 1.0))
    return theta * deg  # [rad] to [deg]


# --- quick tests ---

# deg = 180/np.pi
# p_robot = np.array([0., 0., 0.])
# p_obj   = np.array([10., 0., 0.])                   # 10 m straight ahead

# print("1) Identity attitude, object straight ahead ⇒ θ ≈ 0°")
# print(misalignment_angle(p_robot, [1,0,0,0], p_obj)*deg)

# print("\n2) Robot yawed +45° (π/4), object still at (10,0,0) map ⇒ θ ≈ 45°")
# c = np.cos(np.deg2rad(22.5))
# s = np.sin(np.deg2rad(22.5))
# q_yaw45 = [c, 0, 0, s]                              # yaw +45° quaternion
# print(misalignment_angle(p_robot, q_yaw45, p_obj)*deg)

# print("\n3) Robot pitched 30°, object at +z ⇒ θ ≈ 30°")
# # 30° pitch about +y

# c = np.cos(np.deg2rad(30))
# s = np.sin(np.deg2rad(30))
# q_pitch30 = [c, 0, s, 0]

# print(misalignment_angle(p_robot, q_pitch30, p_obj)*deg)


def test_missallignment_ros():
    # we need:
    # 1. a robot pose in map (p and q)
    # 2. an object pose in map
    cam_opt_axis = np.array([1.0, 0.0, 0.0])  # camera axis in body +x axis


class MissAlignmentTest(Node):
    def __init__(self):
        super().__init__("missalignment_test")

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0,
        )

        self.get_logger().info("MissAlignmentTest node started")

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            f"/fmu/out/vehicle_attitude",
            self.vehicle_attitude_callback,
            qos_profile_sub,
        )

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            f"/fmu/out/vehicle_local_position",
            self.vehicle_local_position_callback,
            qos_profile_sub,
        )

        self.object_pose_sub = self.create_subscription(
            PoseStamped, "/pose/icp_result", self.object_pose_callback, qos_profile_sub
        )

        # Robot pose in map
        self.vehicle_local_position = np.zeros(3)  # robot position in map
        self.vehicle_attitude = np.zeros(4)  # robot attitude in map

        # Object pose in map
        self.p_obj = np.zeros(3)  # object position in map

        # Add TF buffer and listener for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # set timer to call the function every 0.1 seconds
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # check if all the data is not zeros
        if (
            np.count_nonzero(self.vehicle_local_position) > 0
            and np.count_nonzero(self.vehicle_attitude) > 0
            and np.count_nonzero(self.p_obj) > 0
        ):
            # calculate the misalignment angle
            theta = misalignment_angle(
                self.vehicle_local_position, self.vehicle_attitude, self.p_obj
            )
            self.get_logger().info(f"Misalignment angle: {theta:.2f} degrees")

    def object_pose_callback(self, msg: PoseStamped):
        # get only the position of the object, convert into map frame
        # set z to 0 since we are in 2D
        msg.pose.position.z = 0.0
        obj_pose_map = self.transform_pose(msg)
        if obj_pose_map is None:
            self.get_logger().error("Failed to transform object pose")
            return
        self.p_obj = np.array(
            [obj_pose_map.position.x, obj_pose_map.position.y, obj_pose_map.position.z]
        )  # object position in map frame

    def vehicle_attitude_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.vehicle_attitude[0] = msg.q[0]
        self.vehicle_attitude[1] = msg.q[1]
        self.vehicle_attitude[2] = -msg.q[2]
        self.vehicle_attitude[3] = -msg.q[3]

    def vehicle_local_position_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.vehicle_local_position[0] = msg.x
        self.vehicle_local_position[1] = -msg.y
        self.vehicle_local_position[2] = -msg.z
        # self.vehicle_local_velocity[0] = msg.vx
        # self.vehicle_local_velocity[1] = -msg.vy
        # self.vehicle_local_velocity[2] = -msg.vz

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
                obj_map = do_transform_pose(pose_stamped.pose, transform)

                return obj_map

        except Exception as e:
            self.get_logger().error(f"Transform error: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)

    missAlignment_node = MissAlignmentTest()

    rclpy.spin(missAlignment_node)

    missAlignment_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
