#ifndef TRAJECTORY_MOTION_COMMANDS_HPP
#define TRAJECTORY_MOTION_COMMANDS_HPP

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
    class TrajectoryMotion : public as2::motionCommandsHandlers::BasicMotionCommandsHandler
    {
    public:
      TrajectoryMotion(as2::Node *node_ptr);

      bool sendTrajectoryCommandWithYawAngle(
          const float &x, const float &y, const float &z, const float &yaw_angle,
          const float &vx, const float &vy, const float &vz,
          const float &ax, const float &ay, const float &az);

      bool sendTrajectoryCommandWithYawSpeed(
          const float &x, const float &y, const float &z,
          const float &vx, const float &vy, const float &vz, const float &yaw_speed,
          const float &ax, const float &ay, const float &az);

    private:
      as2_msgs::msg::ControllerControlMode ownSetControlMode();
      as2_msgs::msg::ControllerControlMode current_mode_;
    };

  } // namespace motionCommandsHandlers
} // namespace as2

#endif // TRAJECTORY_MOTION_COMMANDS_HPP