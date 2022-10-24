from motion_reference_handlers.basic_motion_references import BasicMotionReferenceHandler
from as2_msgs.msg import ControlMode


class HoverMotion(BasicMotionReferenceHandler):
    def __init__(self, node):
        super().__init__(node)
        self.desired_control_mode_.yaw_mode = ControlMode.NONE
        self.desired_control_mode_.control_mode = ControlMode.HOVER
        self.desired_control_mode_.reference_frame = ControlMode.UNDEFINED_FRAME

    def sendHover(self):
        self.desired_control_mode_.yaw_mode = ControlMode.NONE
        self.desired_control_mode_.control_mode = ControlMode.HOVER
        self.desired_control_mode_.reference_frame = ControlMode.UNDEFINED_FRAME
        return self.checkMode()
