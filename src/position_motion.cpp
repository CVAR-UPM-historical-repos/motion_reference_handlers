/*!*******************************************************************************************
 *  \file       position_motion.cpp
 *  \brief      This file contains the implementation of the PositionMotion class.
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

#include "motion_reference_handlers/position_motion.hpp"

namespace as2
{
    namespace motionReferenceHandlers
    {
        PositionMotion::PositionMotion(as2::Node *node_ptr) : BasicMotionReferenceHandler(node_ptr)
        {
            desired_control_mode_.yaw_mode = as2_msgs::msg::ControlMode::NONE;
            desired_control_mode_.control_mode = as2_msgs::msg::ControlMode::POSITION;
            desired_control_mode_.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
        };

        bool PositionMotion::sendPositionCommandWithYawAngle(
            const float &x, const float &y, const float &z, const float &yaw_angle,
            const float &vx = 0.0f, const float &vy = 0.0f, const float &vz = 0.0f)
        {
            return sendPositionCommandWithYawAngle(
                x, y, z, tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_angle)),
                vx, vy, vz);
        }

        bool PositionMotion::sendPositionCommandWithYawAngle(
            const float &x, const float &y, const float &z, const geometry_msgs::msg::Quaternion &q,
            const float &vx = 0.0f, const float &vy = 0.0f, const float &vz = 0.0f)
        {
            desired_control_mode_.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
            this->command_pose_msg_.pose.position.x = x;
            this->command_pose_msg_.pose.position.y = y;
            this->command_pose_msg_.pose.position.z = z;

            this->command_pose_msg_.pose.orientation = q;

            this->command_twist_msg_.twist.linear.x = vx;
            this->command_twist_msg_.twist.linear.y = vy;
            this->command_twist_msg_.twist.linear.z = vz;

            return this->sendCommand();
        };

        bool PositionMotion::sendPositionCommandWithYawSpeed(
            const float &x, const float &y, const float &z, const float &yaw_speed,
            const float &vx = 0.0f, const float &vy = 0.0f, const float &vz = 0.0f)
        {
            desired_control_mode_.yaw_mode = as2_msgs::msg::ControlMode::YAW_SPEED;

            this->command_pose_msg_.pose.position.x = x;
            this->command_pose_msg_.pose.position.y = y;
            this->command_pose_msg_.pose.position.z = z;

            this->command_twist_msg_.twist.angular.z = yaw_speed;

            this->command_twist_msg_.twist.linear.x = vx;
            this->command_twist_msg_.twist.linear.y = vy;
            this->command_twist_msg_.twist.linear.z = vz;

            return this->sendCommand();
        };
    } // namespace motionReferenceHandlers
} // namespace as2
