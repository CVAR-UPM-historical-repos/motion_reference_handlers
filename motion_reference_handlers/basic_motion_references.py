"""
Implementation of a motion reference handler base.
"""

import time
from rclpy.node import Node, MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from as2_msgs.msg import ControlMode, ControllerInfo
from as2_msgs.srv import SetControlMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from trajectory_msgs.msg import JointTrajectoryPoint

as2_names_topic_motion_reference_qos = qos_profile_sensor_data
as2_names_topic_motion_reference_pose = "motion_reference/pose"
as2_names_topic_motion_reference_twist = "motion_reference/twist"
as2_names_topic_motion_reference_trajectory = "motion_reference/trajectory"
as2_names_topic_controller_qos = qos_profile_system_default
as2_names_topic_controller_info = "controller/info"
as2_names_srv_controller_set_control_mode = "controller/set_control_mode"


class Singleton(type):
    """ Implementation of a singleton class """
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(
                Singleton, cls).__call__(*args, **kwargs)
        else:
            cls._instances[cls].__init__(*args, **kwargs)
        return cls._instances[cls]


class BasicMotionReferencesBase(metaclass=Singleton):
    """ Implementation of a motion reference handler base for singletons """
    number_of_instances_ = -1
    current_mode_ = ControlMode()
    controller_info_sub_ = None
    command_traj_pub_ = None
    command_pose_pub_ = None
    command_twist_pub_ = None

    def __init__(self, node: Node):
        self.number_of_instances_ += 1

        if self.number_of_instances_ == 0:
            my_callback_group = MutuallyExclusiveCallbackGroup()
            self.controller_info_sub_ = node.create_subscription(
                ControllerInfo, as2_names_topic_controller_info,
                self.controller_info_callback, as2_names_topic_controller_qos, callback_group=my_callback_group)

            self.command_pose_pub_ = node.create_publisher(
                PoseStamped, as2_names_topic_motion_reference_pose,
                as2_names_topic_motion_reference_qos)

            self.command_twist_pub_ = node.create_publisher(
                TwistStamped, as2_names_topic_motion_reference_twist,
                as2_names_topic_motion_reference_qos)

            self.command_traj_pub_ = node.create_publisher(
                JointTrajectoryPoint, as2_names_topic_motion_reference_trajectory,
                as2_names_topic_motion_reference_qos)

        node.get_logger().debug(
            f'#{self.number_of_instances_} instances of BasicMotionReferenceHandler created')

    def controller_info_callback(self, msg: ControllerInfo):
        """ Callback for controller info """
        self.current_mode_ = msg.input_control_mode
        return


class BasicMotionReferenceHandler():
    """ Implementation of a motion reference handler base """

    def __init__(self, node: Node):
        self.motion_handler_ = BasicMotionReferencesBase(node)

        self.node = node
        self.command_trajectory_msg_ = JointTrajectoryPoint()
        self.command_pose_msg_ = PoseStamped()
        self.command_twist_msg_ = TwistStamped()
        self.desired_control_mode_ = ControlMode()
        self.desired_control_mode_.yaw_mode = ControlMode.NONE
        self.desired_control_mode_.control_mode = ControlMode.UNSET
        self.desired_control_mode_.reference_frame = ControlMode.UNDEFINED_FRAME

    def check_mode(self) -> bool:
        """ Check if the current mode is the desired mode """
        if (self.desired_control_mode_.control_mode ==
                self.motion_handler_.current_mode_.control_mode) == ControlMode.HOVER:
            return True
        if (self.desired_control_mode_.yaw_mode != self.motion_handler_.current_mode_.yaw_mode or
                self.desired_control_mode_.control_mode != self.motion_handler_.current_mode_.control_mode):
            if not self.set_mode(self.desired_control_mode_):
                return False
        return True

    def send_pose_command(self) -> bool:
        """ Send a pose command """
        if not self.check_mode():
            return False
        self.command_pose_msg_.header.stamp = self.node.get_clock().now().to_msg()
        self.motion_handler_.command_pose_pub_.publish(
            self.command_pose_msg_)
        return True

    def send_twist_command(self) -> bool:
        """ Send a twist command """
        if not self.check_mode():
            return False
        self.command_twist_msg_.header.stamp = self.node.get_clock().now().to_msg()
        self.motion_handler_.command_twist_pub_.publish(
            self.command_twist_msg_)
        return True

    def send_trajectory_command(self) -> bool:
        """ Send a trajectory command """
        if not self.check_mode():
            return False
        self.motion_handler_.command_traj_pub_.publish(
            self.command_trajectory_msg_)
        return True

    def set_mode(self, mode: ControlMode) -> bool:
        """ Set the control mode """
        set_control_mode_cli_ = self.node.create_client(
            SetControlMode, as2_names_srv_controller_set_control_mode)

        if not set_control_mode_cli_.wait_for_service(timeout_sec=3):
            self.node.get_logger().error(
                f"Service {self.node.get_namespace()}/{as2_names_srv_controller_set_control_mode} not available")
            return False

        req = SetControlMode.Request()
        req.control_mode = mode
        resp = set_control_mode_cli_.call(req)
        if resp.success:
            while self.motion_handler_.current_mode_.control_mode != mode.control_mode:
                self.node.get_logger().debug(
                    "Set mode callback: waiting for control mode change confirmation")
                time.sleep(0.1)
            self.node.get_logger().debug("Set control mode success")
            return True

        self.node.get_logger().error("Failed to set control mode")
        return False
