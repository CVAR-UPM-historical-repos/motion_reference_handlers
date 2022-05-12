
#include "motion_reference_handlers/speed_motion.hpp"

namespace as2
{
    namespace motionReferenceHandlers
    {
        SpeedMotion::SpeedMotion(as2::Node *node_ptr) : BasicMotionReferenceHandler(node_ptr)
        {
            desired_control_mode_.yaw_mode = as2_msgs::msg::ControlMode::NONE;
            desired_control_mode_.control_mode = as2_msgs::msg::ControlMode::SPEED;
            desired_control_mode_.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
        };

        bool SpeedMotion::sendSpeedCommandWithYawAngle(
            const float &vx, const float &vy, const float &vz, const float &yaw_angle)
        {
            return sendSpeedCommandWithYawAngle(
                vx, vy, vz, tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_angle)));
        }

        bool SpeedMotion::sendSpeedCommandWithYawAngle(
            const float &vx, const float &vy, const float &vz, const geometry_msgs::msg::Quaternion &q)
        {
            desired_control_mode_.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
            this->command_twist_msg_.twist.linear.x = vx;
            this->command_twist_msg_.twist.linear.y = vy;
            this->command_twist_msg_.twist.linear.z = vz;
            this->command_pose_msg_.pose.orientation = q;

            return this->sendCommand();
        };

        bool SpeedMotion::sendSpeedCommandWithYawSpeed(
            const float &vx, const float &vy, const float &vz, const float &yaw_speed)
        {
            desired_control_mode_.yaw_mode = as2_msgs::msg::ControlMode::YAW_SPEED;
            this->command_twist_msg_.twist.linear.x = vx;
            this->command_twist_msg_.twist.linear.y = vy;
            this->command_twist_msg_.twist.linear.z = vz;
            this->command_twist_msg_.twist.angular.z = yaw_speed;

            return this->sendCommand();
        };
    } // namespace motionReferenceHandlers
} // namespace as2
