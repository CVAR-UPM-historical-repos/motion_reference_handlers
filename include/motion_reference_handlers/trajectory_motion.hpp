/*!*******************************************************************************************
 *  \file       trajectory_motion.hpp
 *  \brief      this file contains the implementation of the TrajectoryMotion class
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef TRAJECTORY_MOTION_COMMANDS_HPP
#define TRAJECTORY_MOTION_COMMANDS_HPP

#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "as2_core/node.hpp"
#include "basic_motion_references.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace as2
{
  namespace motionReferenceHandlers
  {
    class TrajectoryMotion : public as2::motionReferenceHandlers::BasicMotionReferenceHandler
    {
    public:
      TrajectoryMotion(as2::Node *node_ptr);
      ~TrajectoryMotion(){};

    public:
      bool sendTrajectoryCommandWithYawAngle(
          const float &x, const float &y, const float &z, const float &yaw_angle,
          const float &vx, const float &vy, const float &vz,
          const float &ax, const float &ay, const float &az);

      bool sendTrajectoryCommandWithYawAngle(
          const std::vector<double> &positions,
          const std::vector<double> &velocities,
          const std::vector<double> &accelerations);

      bool sendTrajectoryCommandWithYawSpeed(
          const float &x, const float &y, const float &z,
          const float &vx, const float &vy, const float &vz, const float &yaw_speed,
          const float &ax, const float &ay, const float &az);

      bool sendTrajectoryCommandWithYawSpeed(
          const std::vector<double> &positions,
          const std::vector<double> &velocities,
          const std::vector<double> &accelerations);
    };

  } // namespace motionReferenceHandlers
} // namespace as2

#endif // TRAJECTORY_MOTION_COMMANDS_HPP
