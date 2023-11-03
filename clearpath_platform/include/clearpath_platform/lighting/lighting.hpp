/**
 *
 *  \file
 *  \brief      Lighting Class Header
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

#ifndef CLEARPATH_PLATFORM__LIGHTING_HPP_
#define CLEARPATH_PLATFORM__LIGHTING_HPP_

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "clearpath_platform_msgs/msg/lights.hpp"
#include "clearpath_platform_msgs/msg/rgb.hpp"
#include "clearpath_platform_msgs/msg/status.hpp"
#include "clearpath_platform_msgs/msg/power.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

namespace clearpath_lighting
{

static constexpr auto LIGHTING_TIMER_TIMEOUT_MS = 20;
static constexpr auto USER_COMMAND_TIMEOUT_MS = 1000;

enum Platform
{
  DD100,
  DO100,
  DD150,
  DO150,
  R100,
  W200
};

static std::map<std::string, Platform> ClearpathPlatforms
{
  {"DD100", Platform::DD100},
  {"DO100", Platform::DO100},
  {"DD150", Platform::DD150},
  {"DO150", Platform::DO150},
  {"R100", Platform::R100},
  {"W200", Platform::W200},
};

class Lighting : public rclcpp::Node
{
public:
  Lighting();

private:
  void spinOnce();

  void initializePublishers();
  void initializeSubscribers();
  void initializeTimers();

  void startUserTimeoutTimer();

  void cmdLightsCallback(const clearpath_platform_msgs::msg::Lights::SharedPtr msg);
  void statusCallback(const clearpath_platform_msgs::msg::Status::SharedPtr msg);
  void batteryStateCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  // Publishers
  rclcpp::Publisher<clearpath_platform_msgs::msg::Lights>::SharedPtr cmd_lights_pub_;

  // Subscribers
  rclcpp::Subscription<clearpath_platform_msgs::msg::Lights>::SharedPtr cmd_lights_sub_;
  rclcpp::Subscription<clearpath_platform_msgs::msg::Status>::SharedPtr status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr lighting_timer_;
  rclcpp::TimerBase::SharedPtr user_timeout_timer_;

  // Messages
  clearpath_platform_msgs::msg::Lights lights_msg_;

  // Variables
  Platform platform_;
  int num_lights_;
  bool user_commands_allowed_;
};

}  // namespace clearpath_lighting

#endif  // CLEARPATH_PLATFORM__LIGHTING_HPP_
