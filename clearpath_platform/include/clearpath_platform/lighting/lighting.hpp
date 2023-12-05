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

#ifndef CLEARPATH_PLATFORM__LIGHTING__LIGHTING_HPP_
#define CLEARPATH_PLATFORM__LIGHTING__LIGHTING_HPP_

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "clearpath_platform_msgs/msg/lights.hpp"
#include "clearpath_platform_msgs/msg/rgb.hpp"
#include "clearpath_platform_msgs/msg/status.hpp"
#include "clearpath_platform_msgs/msg/power.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "clearpath_platform/lighting/sequence.hpp"
#include "clearpath_platform/lighting/platform.hpp"

namespace clearpath_lighting
{

static constexpr auto LIGHTING_TIMER_TIMEOUT_MS = 50;
static constexpr auto USER_COMMAND_TIMEOUT_MS = 1000;

#define MS_TO_STEPS(ms) (ms / LIGHTING_TIMER_TIMEOUT_MS)

class Lighting : public rclcpp::Node
{

public:
  /** The set of states for which different lighting is provided */
  enum class State
  {
    BatteryFault = 0,
    ShoreFault,
    //PumaFault,
    ShoreAndCharged,
    ShoreAndCharging,
    ShorePower,
    Charged,
    Charging,
    Stopped,
    NeedsReset,
    LowBattery,
    Driving,
    Idle
  };

  std::map<State, Sequence> lighting_sequence_;

  Lighting();

private:
  void spinOnce();

  void initializePublishers();
  void initializeSubscribers();
  void initializeTimers();

  void startUserTimeoutTimer();

  void cmdLightsCallback(const clearpath_platform_msgs::msg::Lights::SharedPtr msg);
  void statusCallback(const clearpath_platform_msgs::msg::Status::SharedPtr msg);
  void powerCallback(const clearpath_platform_msgs::msg::Power::SharedPtr msg);
  void batteryStateCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void stopEngagedCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /** Updates the current lighting state based on all inputs */
  void setState(Lighting::State new_state);
  void updateState();

  // Publishers
  rclcpp::Publisher<clearpath_platform_msgs::msg::Lights>::SharedPtr cmd_lights_pub_;

  // Subscribers
  rclcpp::Subscription<clearpath_platform_msgs::msg::Lights>::SharedPtr cmd_lights_sub_;
  rclcpp::Subscription<clearpath_platform_msgs::msg::Status>::SharedPtr status_sub_;
  rclcpp::Subscription<clearpath_platform_msgs::msg::Power>::SharedPtr power_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_engaged_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr lighting_timer_;
  rclcpp::TimerBase::SharedPtr user_timeout_timer_;

  // Messages
  clearpath_platform_msgs::msg::Lights lights_msg_;
  clearpath_platform_msgs::msg::Status status_msg_;
  clearpath_platform_msgs::msg::Power power_msg_;
  sensor_msgs::msg::BatteryState battery_state_msg_;
  std_msgs::msg::Bool stop_engaged_msg_;
  geometry_msgs::msg::Twist cmd_vel_msg_;

  // Variables
  Platform platform_;
  State state_, old_state_;
  int num_lights_;
  bool user_commands_allowed_;
  Sequence current_sequence_;
};

}  // namespace clearpath_lighting

#endif  // CLEARPATH_PLATFORM__LIGHTING__LIGHTING_HPP_
