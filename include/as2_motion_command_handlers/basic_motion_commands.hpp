#ifndef BASIC_MOTION_COMMANDS_HPP
#define BASIC_MOTION_COMMANDS_HPP

#include <functional>
#include <memory>
#include <thread>

#include "as2_core/node.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/names/services.hpp"

#include "as2_msgs/srv/set_controller_control_mode.hpp"
#include <as2_msgs/msg/controller_info.hpp>
#include <as2_msgs/msg/control_mode.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "rclcpp/rclcpp.hpp"

#define AUX_NODE_SPIN_RATE 10

namespace as2
{
    namespace motionCommandsHandlers
    {
        class BasicMotionCommandsHandler
        {
        public:
            BasicMotionCommandsHandler(as2::Node *as2_ptr);
            ~BasicMotionCommandsHandler();

        protected:
            as2::Node *node_ptr_;

            trajectory_msgs::msg::JointTrajectoryPoint command_trajectory_msg_;
            geometry_msgs::msg::PoseStamped command_pose_msg_;
            geometry_msgs::msg::TwistStamped command_twist_msg_;

            as2_msgs::msg::ControlMode desired_control_mode_;

            bool sendCommand();

        private:

            static int number_of_instances_;

            static rclcpp::Client<as2_msgs::srv::SetControllerControlMode>::SharedPtr set_mode_client_;
            static rclcpp::Subscription<as2_msgs::msg::ControllerInfo>::SharedPtr controller_info_sub_;
            static as2_msgs::msg::ControlMode current_mode_;

            rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr command_traj_pub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr command_pose_pub_;
            rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_twist_pub_;

            bool setMode(const as2_msgs::msg::ControlMode &mode);
            void publishCommands();
        };

    } // namespace motionCommandsHandlers
} // namespace as2

#endif // BASIC_MOTION_COMMANDS_HPP
