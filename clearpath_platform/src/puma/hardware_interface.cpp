/**
Software License Agreement (BSD)

\file      hardware_interface.cpp
\authors   Luis Camero <lcamero@clearpathrobotics.com>
\copyright Copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of Clearpath Robotics nor the names of its contributors
  may be used to endorse or promote products derived from this software
  without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/
#include "clearpath_platform/puma/hardware_interface.hpp"

using clearpath_platform::PumaHardwareInterface;

/**
 * @brief Construct a new PumaHardwareInterface object
*/
PumaHardwareInterface::PumaHardwareInterface(std::string node_name)
: Node(node_name)
{
  sub_feedback_ = create_subscription<puma_motor_msgs::msg::MultiFeedback>(
    "platform/puma/feedback",
    rclcpp::SensorDataQoS(),
    std::bind(&PumaHardwareInterface::feedback_callback, this, std::placeholders::_1));

  pub_cmd_ = create_publisher<sensor_msgs::msg::JointState>(
    "platform/puma/cmd",
    rclcpp::SensorDataQoS());
}

/**
 * @brief Callback for feedback
 *
 * @param msg
*/
void PumaHardwareInterface::feedback_callback(const puma_motor_msgs::msg::MultiFeedback::SharedPtr msg)
{
  feedback_ = *msg;
  has_feedback_ = true;
}

/**
 * @brief Publish drive command
 *
 * @param
*/
void PumaHardwareInterface::drive_command(const sensor_msgs::msg::JointState msg)
{
  pub_cmd_->publish(msg);
  return;
}

/**
 * @brief Check if there is new feedback
 *
 * @return true
 * @return false
*/
bool PumaHardwareInterface::has_new_feedback()
{
  return has_feedback_;
}

/**
 * @brief Get feedback
 *
 * @return puma_motor_msgs::msg::MultiFeedback
*/
puma_motor_msgs::msg::MultiFeedback PumaHardwareInterface::get_feedback()
{
  has_feedback_ = false;
  return feedback_;
}
