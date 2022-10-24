import rclpy
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation


def get_quaternion_from_yaw_angle(yaw_angle):
    rot = Rotation.from_euler(
        'xyz', [0.0, 0.0, yaw_angle], degrees=True)
    rot_quat = rot.as_quat()
    return Quaternion(
        x=rot_quat[0], y=rot_quat[1], z=rot_quat[2], w=rot_quat[3])


def generate_tf_name(namespace, frame_name):
    if len(frame_name) == 0:
        raise RuntimeError("Empty frame name")
    if frame_name[0] == '/':
        return frame_name[1:]
    if len(namespace) == 0:
        rclpy.logging.get_logger("tf_utils").warn(
            "The frame name [%s] is not absolute and the node namespace is empty. This could "
            "lead to conflicts.", frame_name)
        return frame_name
    if namespace[0] == '/':
        return namespace[1:] + "/" + frame_name
    return namespace + "/" + frame_name


def get_tf_name(node, frame_name):
    return generate_tf_name(node.get_namespace(), frame_name)
