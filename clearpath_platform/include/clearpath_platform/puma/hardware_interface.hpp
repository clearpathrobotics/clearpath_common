/**
 *
 *  \file
 *  \brief      Puma Motor hardware interface class
 *  \author     Luis Camero <lcamero@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2024, Clearpath Robotics, Inc.
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
#ifndef CLEARPATH_PLATFORM__PUMA_DRIVE_HARDWARE_INTERFACE_HPP_
#define CLEARPATH_PLATFORM__PUMA_DRIVE_HARDWARE_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "puma_motor_msgs/msg/feedback.hpp"
#include "puma_motor_msgs/msg/multi_feedback.hpp"

namespace clearpath_platform
{

class PumaHardwareInterface
: public rclcpp::Node
{
  public:
  explicit PumaHardwareInterface(std::string node_name);

  void drive_command(const sensor_msgs::msg::JointState msg);

  bool has_new_feedback();
  void feedback_callback(const puma_motor_msgs::msg::MultiFeedback::SharedPtr msg);
  puma_motor_msgs::msg::MultiFeedback get_feedback();

  private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_cmd_;
  rclcpp::Subscription<puma_motor_msgs::msg::MultiFeedback>::SharedPtr sub_feedback_;

  puma_motor_msgs::msg::MultiFeedback feedback_;
  std::atomic_bool has_feedback_;
};

} // namespace clearpath_platform

#endif // CLEARPATH_PLATFORM__PUMA_DRIVE_HARDWARE_INTERFACE_HPP_
