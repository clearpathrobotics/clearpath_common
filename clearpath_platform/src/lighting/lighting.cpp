/**
 *
 *  \file
 *  \brief      Lighting Class
 *  \author     Roni Kreinin <rkreinin@clearpathrobotics.com>
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

#include "clearpath_platform/lighting/lighting.hpp"

using clearpath_lighting::Lighting;

Lighting::Lighting()
: Node("clearpath_lighting"),
  num_lights_(4),
  user_commands_allowed_(false)
{
  this->declare_parameter("platform", "DD100");
  std::string platform = this->get_parameter("platform").as_string();

  try
  {
    platform_ = ClearpathPlatforms.at(platform);
  }
  catch(const std::out_of_range& e)
  {
    throw std::out_of_range("Invalid Platform " + platform);
  }
  
  RCLCPP_INFO(this->get_logger(), "Lighting Platform %s", platform.c_str());

  if (platform_ == Platform::R100)
  {
    num_lights_ = 8;
  }

  lights_msg_ = clearpath_platform_msgs::msg::Lights();
  lights_msg_.lights.resize(num_lights_);

  initializePublishers();
  initializeSubscribers();
  initializeTimers();
}

void Lighting::spinOnce()
{
  // If user is not commanding lights, update lights
  if (!user_commands_allowed_ || !user_timeout_timer_->is_canceled())
  {
    cmd_lights_pub_->publish(lights_msg_);
  }
}

void Lighting::initializePublishers()
{
  cmd_lights_pub_ = this->create_publisher<clearpath_platform_msgs::msg::Lights>(
    "platform/mcu/_cmd_lights",
    rclcpp::SensorDataQoS());
}

void Lighting::initializeSubscribers()
{
  // User command lights
  cmd_lights_sub_ = this->create_subscription<clearpath_platform_msgs::msg::Lights>(
    "platform/cmd_lights",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&Lighting::cmdLightsCallback, this, std::placeholders::_1));

  // MCU status
  status_sub_ = this->create_subscription<clearpath_platform_msgs::msg::Status>(
    "platform/mcu/status",
    rclcpp::SensorDataQoS(),
    std::bind(&Lighting::statusCallback, this, std::placeholders::_1));

  // MCU power status
  battery_state_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    "platform/bms/state",
    rclcpp::SensorDataQoS(),
    std::bind(&Lighting::batteryStateCallback, this, std::placeholders::_1));
}

void Lighting::initializeTimers()
{
  lighting_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(LIGHTING_TIMER_TIMEOUT_MS),
    [this]() -> void
    {
      this->spinOnce();
    }
  );
}

void Lighting::startUserTimeoutTimer()
{
  if (user_timeout_timer_ == nullptr || user_timeout_timer_->is_canceled())
  {
    user_timeout_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(USER_COMMAND_TIMEOUT_MS),
      [this]() -> void
      {
        RCLCPP_INFO(this->get_logger(), "User command timeout");
        user_timeout_timer_->cancel();
      }
    );
  }
}

void Lighting::cmdLightsCallback(const clearpath_platform_msgs::msg::Lights::SharedPtr msg)
{
  // Do nothing if user commands are not allowed
  if (!user_commands_allowed_)
  {
    return;
  }

  // Reset timer if it is running
  if (user_timeout_timer_)
  {
    user_timeout_timer_->cancel();
  }

  startUserTimeoutTimer();

  // Publish if allowed
  cmd_lights_pub_->publish(*msg);
}

void Lighting::statusCallback(const clearpath_platform_msgs::msg::Status::SharedPtr msg)
{
  (void)msg;
}

void Lighting::batteryStateCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  (void)msg;
}