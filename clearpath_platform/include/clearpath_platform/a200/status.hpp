/**
Software License Agreement (BSD)
\file      status.hpp
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

#ifndef CLEARPATH_PLATFORM__A200_STATUS_HPP
#define CLEARPATH_PLATFORM__A200_STATUS_HPP

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "clearpath_platform_msgs/msg/power.hpp"
#include "clearpath_platform_msgs/msg/status.hpp"
#include "clearpath_platform_msgs/msg/stop_status.hpp"

namespace a200_status
{

class A200Status
: public rclcpp::Node
{
  public:
  explicit A200Status();

  void publish_power(const clearpath_platform_msgs::msg::Power & power_msg);
  void publish_status(const clearpath_platform_msgs::msg::Status & status_msg);
  void publish_stop_status(const clearpath_platform_msgs::msg::StopStatus & stop_status_msg);
  void publish_stop_state(const std_msgs::msg::Bool & stop_msg);
  void publish_temps(const std_msgs::msg::Float32 & driver_left_msg, 
        const std_msgs::msg::Float32 & driver_right_msg,
        const std_msgs::msg::Float32 & motor_left_msg,
        const std_msgs::msg::Float32 & motor_right_msg);

  private:
  rclcpp::Publisher<clearpath_platform_msgs::msg::Power>::SharedPtr pub_power_;
  rclcpp::Publisher<clearpath_platform_msgs::msg::Status>::SharedPtr pub_status_;
  rclcpp::Publisher<clearpath_platform_msgs::msg::StopStatus>::SharedPtr pub_stop_status_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stop_state_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_driver_left_temp_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_driver_right_temp_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_motor_left_temp_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_motor_right_temp_;
};

}

#endif  // CLEARPATH_PLATFORM__A200_STATUS_HPP