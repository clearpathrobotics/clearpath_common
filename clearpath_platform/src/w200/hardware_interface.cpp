/**
 *
 *  \file
 *  \brief      Class representing W200 hardware interface
 *  \author     Roni Kreinin <rkreinin@clearpathrobotics.com>
 *  \author     Tony Baltovski <tbaltovski@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2023, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#include "clearpath_platform/w200/hardware_interface.hpp"

using clearpath_platform::W200HardwareInterface;

/**
 * @brief Construct a new W200HardwareInterface object
 * 
 */
W200HardwareInterface::W200HardwareInterface(std::string node_name)
: Node(node_name)
{
  sub_left_feedback_ = create_subscription<std_msgs::msg::Float64>(
    "platform/motor/left/status/velocity",
    rclcpp::SensorDataQoS(),
    std::bind(&W200HardwareInterface::feedback_left_callback, this, std::placeholders::_1));

  sub_right_feedback_ = create_subscription<std_msgs::msg::Float64>(
    "platform/motor/right/status/velocity",
    rclcpp::SensorDataQoS(),
    std::bind(&W200HardwareInterface::feedback_right_callback, this, std::placeholders::_1));

  pub_left_cmd = create_publisher<std_msgs::msg::Float64>(
    "platform/motor/left/cmd_velocity",
    rclcpp::SensorDataQoS());

  pub_right_cmd = create_publisher<std_msgs::msg::Float64>(
    "platform/motor/right/cmd_velocity",
    rclcpp::SensorDataQoS());
}

/**
 * @brief Callback for left feedback
 *
 * @param msg
 */
void W200HardwareInterface::feedback_left_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  feedback_left_ = *msg;
  has_left_feedback_ = true;
}

/**
 * @brief Callback for right feedback
 *
 * @param msg
 */
void W200HardwareInterface::feedback_right_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  feedback_right_ = *msg;
  has_right_feedback_ = true;
}

/**
 * @brief Publish Drive message
 * 
 * @param left_wheel Left wheel command
 * @param right_wheel Right wheel command
 * @param mode Command mode
 */
void W200HardwareInterface::drive_command(const float & left_wheel, const float & right_wheel)
{
  std_msgs::msg::Float64 msg_left_cmd, msg_right_cmd;
  msg_left_cmd.data = left_wheel;
  msg_right_cmd.data = right_wheel;
  pub_left_cmd->publish(msg_left_cmd);
  pub_right_cmd->publish(msg_right_cmd);
}

/**
 * @brief Check if there is new feedback on both left and right side
 *
 * @return true
 * @return false
 */
bool W200HardwareInterface::has_new_feedback()
{
  return (has_left_feedback_ && has_right_feedback_);
}

/**
 * @brief Get the left side feedback
 *
 * @return std_msgs::msg::Float64
 */
std_msgs::msg::Float64 W200HardwareInterface::get_left_feedback()
{
  has_left_feedback_ = false;
  return feedback_left_;
}

/**
 * @brief Get the right side feedback
 *
 * @return std_msgs::msg::Float64
 */
std_msgs::msg::Float64 W200HardwareInterface::get_right_feedback()
{
  has_right_feedback_ = false;
  return feedback_right_;
}
