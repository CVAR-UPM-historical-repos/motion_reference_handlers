#ifndef SPEED_MOTION_COMMANDS_HPP
#define SPEED_MOTION_COMMANDS_HPP

#include <functional>
#include <memory>
#include <thread>

#include "as2_core/node.hpp"
#include "basic_motion_references.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace as2
{
  namespace motionReferenceHandlers
  {
    class SpeedMotion : public as2::motionReferenceHandlers::BasicMotionReferenceHandler
    {
    public:
      SpeedMotion(as2::Node *node_ptr);
      ~SpeedMotion(){};

    public:
      bool sendSpeedCommandWithYawAngle(
          const float &vx, const float &vy, const float &vz, const float &yaw_angle);
      bool sendSpeedCommandWithYawAngle(
          const float &vx, const float &vy, const float &vz, const geometry_msgs::msg::Quaternion &q);
      bool sendSpeedCommandWithYawSpeed(
          const float &vx, const float &vy, const float &vz, const float &yaw_speed);
    };

  } // namespace motionReferenceHandlers
} // namespace as2

#endif // SPEED_MOTION_COMMANDS_HPP
