
/**
Software License Agreement (BSD)
\file      a200_status.cpp
\authors   Tony Baltovski <tbaltovski@clearpathrobotics.com>
\copyright Copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the 
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include "clearpath_platform/a200/status.hpp"

/**
 * @brief Construct a new A200Status object
 * 
 */
a200_status::A200Status::A200Status()
: Node("a200_status_node")
{
  pub_power_= create_publisher<clearpath_platform_msgs::msg::Power>(
    "platform/mcu/status/power",
    rclcpp::SensorDataQoS());

  pub_status_= create_publisher<clearpath_platform_msgs::msg::Status>(
    "platform/mcu/status",
    rclcpp::SensorDataQoS());

//   pub_stop_status_= create_publisher<clearpath_platform_msgs::msg::StopStatus>(
//     "platform/mcu/status/stop",
//     rclcpp::SensorDataQoS());

  pub_stop_state_= create_publisher<std_msgs::msg::Bool>(
    "platform/emergency_stop",
    rclcpp::SensorDataQoS());

  pub_driver_left_temp_ = create_publisher<std_msgs::msg::Float32>(
    "platform/driver/left/temperature",
    rclcpp::SensorDataQoS());

  pub_driver_right_temp_ = create_publisher<std_msgs::msg::Float32>(
    "platform/driver/right/temperature",
    rclcpp::SensorDataQoS());

  pub_motor_left_temp_ = create_publisher<std_msgs::msg::Float32>(
    "platform/motors/left/temperature",
    rclcpp::SensorDataQoS());

  pub_motor_right_temp_ = create_publisher<std_msgs::msg::Float32>(
    "platform/motors/right/temperature",
    rclcpp::SensorDataQoS());
}


/**
 * @brief Publish Power Message
 * 
 * @param power_msg Message to publish
 */
void a200_status::A200Status::publish_power(const clearpath_platform_msgs::msg::Power & power_msg)
{
  pub_power_->publish(power_msg);
}

/**
 * @brief Publish Status Message
 * 
 * @param status_msg Message to publish
 */
void a200_status::A200Status::publish_status(const clearpath_platform_msgs::msg::Status & status_msg)
{
  pub_status_->publish(status_msg);
}

/**
 * @brief Publish StopStatus Message
 * 
 * @param stop_status_msg Message to publish
 */
void a200_status::A200Status::publish_stop_status(const clearpath_platform_msgs::msg::StopStatus & stop_status_msg)
{
  pub_stop_status_->publish(stop_status_msg);
}

/**
 * @brief Publish Stop Message
 * 
 * @param stop_msg Message to publish
 */
void a200_status::A200Status::publish_stop_state(const std_msgs::msg::Bool & stop_msg)
{
  pub_stop_state_->publish(stop_msg);
}

/**
 * @brief Publish Temperature Messages
 * 
 * @param driver_left_msg Driver left message to publish
 * @param driver_right_msg Driver right message to publish
 * @param motor_left_msg Motor left message to publish
 * @param motor_right_msg Motor right message to publish
 */
void a200_status::A200Status::publish_temps(const std_msgs::msg::Float32 & driver_left_msg, 
    const std_msgs::msg::Float32 & driver_right_msg,
    const std_msgs::msg::Float32 & motor_left_msg,
    const std_msgs::msg::Float32 & motor_right_msg)
{
  pub_driver_left_temp_->publish(driver_left_msg);
  pub_driver_right_temp_->publish(driver_right_msg);
  pub_motor_left_temp_->publish(motor_left_msg);
  pub_motor_right_temp_->publish(motor_right_msg);
}