
#include "as2_motion_command_handlers/speed_motion.hpp"

namespace as2
{
    namespace motionCommandsHandlers
    {
        SpeedMotion::SpeedMotion(as2::Node *node_ptr) : BasicMotionCommandsHandler(node_ptr){};

        bool SpeedMotion::sendSpeedCommandWithYawAngle(
            const float &vx, const float &vy, const float &vz, const float &yaw_angle)
        {
            return sendSpeedCommandWithYawAngle(
                vx, vy, vz, tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_angle)));
        }

        bool SpeedMotion::sendSpeedCommandWithYawAngle(
            const float &vx, const float &vy, const float &vz, const geometry_msgs::msg::Quaternion &q)
        {
            current_mode_.yaw_mode = as2_msgs::msg::ControllerControlMode::YAW_ANGLE;
            this->command_twist_msg_.twist.linear.x = vx;
            this->command_twist_msg_.twist.linear.y = vy;
            this->command_twist_msg_.twist.linear.z = vz;
            this->command_pose_msg_.pose.orientation = q;

            return this->sendCommand();
        };

        bool SpeedMotion::sendSpeedCommandWithYawSpeed(
            const float &vx, const float &vy, const float &vz, const float &yaw_speed)
        {
            current_mode_.yaw_mode = as2_msgs::msg::ControllerControlMode::YAW_SPEED;
            this->command_twist_msg_.twist.linear.x = vx;
            this->command_twist_msg_.twist.linear.y = vy;
            this->command_twist_msg_.twist.linear.z = vz;
            this->command_twist_msg_.twist.angular.z = yaw_speed;

            return this->sendCommand();
        };

        as2_msgs::msg::ControllerControlMode SpeedMotion::ownSetControlMode()
        {
            as2_msgs::msg::ControllerControlMode control_mode_msg;

            control_mode_msg.control_mode = as2_msgs::msg::ControllerControlMode::SPEED_MODE;
            if (current_mode_.yaw_mode == as2_msgs::msg::ControllerControlMode::YAW_ANGLE)
            {
                control_mode_msg.yaw_mode = as2_msgs::msg::ControllerControlMode::YAW_ANGLE;
            }
            else
            {
                control_mode_msg.yaw_mode = as2_msgs::msg::ControllerControlMode::YAW_SPEED;
            }
            return control_mode_msg;
        };

    } // namespace motionCommandsHandlers
} // namespace as2
