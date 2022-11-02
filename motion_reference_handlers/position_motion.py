from motion_reference_handlers.basic_motion_references import BasicMotionReferenceHandler
import motion_reference_handlers.utils as utils
from as2_msgs.msg import ControlMode
from geometry_msgs.msg import PoseStamped, TwistStamped


class PositionMotion(BasicMotionReferenceHandler):
    def __init__(self, node):
        super().__init__(node)
        self.desired_control_mode_.yaw_mode = ControlMode.NONE
        self.desired_control_mode_.control_mode = ControlMode.POSITION
        self.desired_control_mode_.reference_frame = ControlMode.UNDEFINED_FRAME

    def __own_send_command(self, yaw_mode, pose_msg, twist_mgs):
        self.desired_control_mode_.yaw_mode = yaw_mode
        self.command_pose_msg_ = pose_msg
        self.command_twist_msg_ = twist_mgs
        send_pose = self.sendPoseCommand()
        send_twist = self.sendTwistCommand()
        return send_pose and send_twist

    def __check_input_pose(self, pose, pose_frame_id):
        pose_msg = PoseStamped()
        if isinstance(pose, list):
            pose_msg.pose.position.x = float(pose[0])
            pose_msg.pose.position.y = float(pose[1])
            pose_msg.pose.position.z = float(pose[2])
            pose_msg.header.frame_id = pose_frame_id
        elif isinstance(pose, PoseStamped):
            pose_msg = pose
        else:
            self.node.get_logger().error("Pose is not a list or PoseStamped")
            return None

        if pose_msg.header.frame_id == '':
            self.node.get_logger().error("Pose frame id is not set")
            return None
        return pose_msg

    def __check_input_twist(self, twist, twist_frame_id):
        twist_mgs = TwistStamped()

        if isinstance(twist, float):
            twist_mgs.twist.linear.x = twist
            twist_mgs.twist.linear.y = twist
            twist_mgs.twist.linear.z = twist
            twist_mgs.header.frame_id = twist_frame_id
        elif isinstance(twist, list):
            twist_mgs.twist.linear.x = float(twist[0])
            twist_mgs.twist.linear.y = float(twist[1])
            twist_mgs.twist.linear.z = float(twist[2])
            twist_mgs.header.frame_id = twist_frame_id
        elif isinstance(twist, TwistStamped):
            twist_mgs = twist
        elif twist == None:
            # Default value -> no limit
            twist_mgs.header.frame_id = 'earth'
        else:
            self.node.get_logger().error("Twist is not a float, list or TwistStamped")
            return None

        if twist_mgs.header.frame_id == '':
            self.node.get_logger().error("Twist frame id is not set")
            return None

        return twist_mgs

    def send_position_command_with_yaw_angle(self, pose, twist_limit=None, pose_frame_id='', twist_frame_id='', yaw_angle=None):
        pose_msg = self.__check_input_pose(pose, pose_frame_id)
        twist_msg = self.__check_input_twist(twist_limit, twist_frame_id)

        if twist_msg == None or pose_msg == None:
            return False

        if isinstance(pose, list):
            if not isinstance(yaw_angle, float):
                self.node.get_logger().error(
                    "Yaw angle is not set")
                return False
            pose_msg.pose.orientation = utils.get_quaternion_from_yaw_angle(
                yaw_angle)

        return self.__own_send_command(ControlMode.YAW_ANGLE, pose_msg, twist_msg)

    def send_position_command_with_yaw_speed(self, pose, twist_limit=None, pose_frame_id='', twist_frame_id='', yaw_speed=None):
        pose_msg = self.__check_input_pose(pose, pose_frame_id)
        twist_msg = self.__check_input_twist(twist_limit, twist_frame_id)

        if twist_msg == None or pose_msg == None:
            return False

        if isinstance(twist_limit, list):
            if not isinstance(yaw_speed, float):
                self.node.get_logger().error(
                    "Yaw speed is not set")
                return False

        if yaw_speed != None:
            twist_msg.twist.angular.z = yaw_speed

        return self.__own_send_command(ControlMode.YAW_SPEED, pose_msg, twist_msg)
