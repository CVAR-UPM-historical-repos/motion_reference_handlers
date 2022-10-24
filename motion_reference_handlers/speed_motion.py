from motion_reference_handlers.basic_motion_references import BasicMotionReferenceHandler
import motion_reference_handlers.utils as utils
from as2_msgs.msg import ControlMode
from geometry_msgs.msg import PoseStamped, TwistStamped


class SpeedMotion(BasicMotionReferenceHandler):
    def __init__(self, node):
        super().__init__(node)
        self.desired_control_mode_.yaw_mode = ControlMode.NONE
        self.desired_control_mode_.control_mode = ControlMode.SPEED
        self.desired_control_mode_.reference_frame = ControlMode.LOCAL_ENU_FRAME

    def __own_send_command(self, yaw_mode, twist_mgs, pose_msg=None):
        self.desired_control_mode_.yaw_mode = yaw_mode

        send_pose = True
        if pose_msg != None:
            self.command_pose_msg_ = pose_msg
            send_pose = self.sendPoseCommand()

        self.command_twist_msg_ = twist_mgs
        send_twist = self.sendTwistCommand()
        return send_pose and send_twist

    def __check_input_twist(self, twist, twist_frame_id):
        twist_mgs = TwistStamped()

        if isinstance(twist, list):
            twist_mgs.twist.linear.x = float(twist[0])
            twist_mgs.twist.linear.y = float(twist[1])
            twist_mgs.twist.linear.z = float(twist[2])
            twist_mgs.header.frame_id = twist_frame_id
        elif isinstance(twist, TwistStamped):
            twist_mgs = twist
        else:
            self.node.get_logger().error("Twist is not a list or TwistStamped")
            return None

        if twist_mgs.header.frame_id == '':
            self.node.get_logger().error("Twist frame id is not set")
            return None

        return twist_mgs

    def send_speed_command_with_yaw_angle(self, twist, pose=None, twist_frame_id='', yaw_angle=None, pose_frame_id='earth',):
        twist_msg = self.__check_input_twist(twist, twist_frame_id)

        if twist_msg == None:
            return False

        pose_msg = PoseStamped()
        if isinstance(pose, PoseStamped):
            pose_msg = pose
        elif isinstance(yaw_angle, float):
            pose_msg.header.frame_id = pose_frame_id
            pose_msg.pose.orientation = utils.get_quaternion_from_yaw_angle(
                yaw_angle)
        else:
            self.node.get_logger().error(
                "Yaw angle is not set")
            return False

        return self.__own_send_command(ControlMode.YAW_ANGLE, twist_msg, pose_msg)

    def send_speed_command_with_yaw_speed(self, twist, twist_frame_id='', yaw_speed=None):
        twist_msg = self.__check_input_twist(twist, twist_frame_id)

        if twist_msg == None:
            return False

        if isinstance(twist, list):
            if not isinstance(yaw_speed, float):
                self.node.get_logger().error(
                    "Yaw speed is not set")
                return False
            twist_msg.twist.angular.z = yaw_speed

        return self.__own_send_command(ControlMode.YAW_SPEED, twist_msg)
