#ifndef SPEED_MOTION_COMMANDS_HPP
#define SPEED_MOTION_COMMANDS_HPP

#include <functional>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <thread>

#include "as2_core/node.hpp"
#include "basic_motion_commands.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace as2
{
  namespace motionCommandsHandlers
  {
    class SpeedMotion : public as2::motionCommandsHandlers::BasicMotionCommandsHandler
    {
    public:
      SpeedMotion(as2::Node *node_ptr);

      bool sendSpeedCommandWithYawAngle(
          const float &vx, const float &vy, const float &vz, const float &yaw_angle);
      bool sendSpeedCommandWithYawAngle(
          const float &vx, const float &vy, const float &vz, const geometry_msgs::msg::Quaternion &q);
      bool sendSpeedCommandWithYawSpeed(
          const float &vx, const float &vy, const float &vz, const float &yaw_speed);

    private:
      as2_msgs::msg::ControllerControlMode ownSetControlMode();
      as2_msgs::msg::ControllerControlMode current_mode_;
    };

  } // namespace motionCommandsHandlers
} // namespace as2

#endif // SPEED_MOTION_COMMANDS_HPP