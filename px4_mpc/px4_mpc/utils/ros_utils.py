import numpy as np
import tf_transformations as tft
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import rclpy
import tf_transformations as tft


def pose_to_matrix(pose):
    """Convert geometry_msgs/Pose to 4×4 homogeneous matrix."""
    q = pose.orientation
    p = pose.position
    T = np.eye(4)
    T[:3, :3] = tft.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def matrix_to_posestamped(T, lookup_transform, target_frame, stamp):
    """Convert 4×4 homogeneous matrix `T` to geometry_msgs/PoseStamped."""

    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = target_frame

    # convert the matrix to Pose message
    q = tft.quaternion_from_matrix(T)  # (x,y,z,w)
    p = T[:3, 3]
    q_z180 = tft.quaternion_from_euler(0, 0, np.pi)  # 180 degrees around Z axis
    q_goal = tft.quaternion_multiply(q, q_z180)

    msg.pose.position.x = p[0]
    msg.pose.position.y = p[1]
    msg.pose.position.z = p[2]
    msg.pose.orientation.x = q_goal[0]
    msg.pose.orientation.y = q_goal[1]
    msg.pose.orientation.z = q_goal[2]
    msg.pose.orientation.w = q_goal[3]

    # Apply the transform to get pose in map frame
    msg.pose = do_transform_pose(msg.pose, lookup_transform)

    return msg


def offset_in_front(T_cam_obj, offset):
    """
    Return T_cam_goal = T_cam_obj  ·  T_offset(+X).

    Parameters
    ----------
    T_cam_obj : (4,4) ndarray – camera ➜ object transform
    offset    : float        – metres to move along +X of the object
    """
    T_offset = np.eye(4)
    T_offset[0, 3] = offset  # +X in object frame
    return T_cam_obj @ T_offset  # matrix product
