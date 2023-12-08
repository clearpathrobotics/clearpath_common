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
#include "clearpath_platform/lighting/color.hpp"

using clearpath_lighting::Lighting;
using Lights = clearpath_platform_msgs::msg::Lights;
using RGB = clearpath_platform_msgs::msg::RGB;

/**
 * @brief Construct a new Lighting node
 */
Lighting::Lighting()
: Node("clearpath_lighting"),
  state_(State::Idle),
  old_state_(State::Idle),
  user_commands_allowed_(false)
{
  // Get platform model from platform parameter
  this->declare_parameter("platform", "dd100");
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

  lights_msg_ = Lights();

  // Set lighting sequences for each state
  lighting_sequence_ = std::map<State, Sequence>{
    {State::BatteryFault, BlinkSequence(
      Sequence::fillLightingState(COLOR_YELLOW, platform_),
      Sequence::fillLightingState(COLOR_RED, platform_),
      MS_TO_STEPS(2000), 0.5)},

    {State::ShoreFault, BlinkSequence(
      Sequence::fillLightingState(COLOR_BLUE, platform_),
      Sequence::fillLightingState(COLOR_RED, platform_),
      MS_TO_STEPS(2000), 0.5)},

    {State::ShoreAndCharging, PulseSequence(
      Sequence::fillLightingState(COLOR_BLUE, platform_),
      Sequence::fillLightingState(COLOR_YELLOW, platform_),
      MS_TO_STEPS(12000))},
    
    {State::ShoreAndCharged, PulseSequence(
      Sequence::fillLightingState(COLOR_BLUE, platform_),
      Sequence::fillLightingState(COLOR_GREEN, platform_),
      MS_TO_STEPS(12000))},
    
    {State::ShorePower, SolidSequence(
      Sequence::fillLightingState(COLOR_BLUE, platform_))},
    
    {State::Charging, PulseSequence(
      Sequence::fillLightingState(COLOR_GREEN, platform_),
      Sequence::fillLightingState(COLOR_YELLOW, platform_),
      MS_TO_STEPS(6000))},
    
    {State::Charged, SolidSequence(
      Sequence::fillLightingState(COLOR_GREEN, platform_))},
    
    {State::Stopped, BlinkSequence(
      Sequence::fillLightingState(COLOR_RED, platform_),
      Sequence::fillLightingState(COLOR_BLACK, platform_),
      MS_TO_STEPS(1000), 0.5)},
    
    {State::NeedsReset, BlinkSequence(
      Sequence::fillLightingState(COLOR_ORANGE, platform_),
      Sequence::fillLightingState(COLOR_RED, platform_),
      MS_TO_STEPS(2000), 0.5)},
    
    {State::LowBattery, PulseSequence(
      Sequence::fillLightingState(COLOR_ORANGE, platform_),
      Sequence::fillLightingState(COLOR_BLACK, platform_),
      MS_TO_STEPS(4000))},

    {State::Driving, SolidSequence(
      Sequence::fillFrontRearLightingState(COLOR_WHITE, COLOR_RED, platform_))},

    {State::Idle, SolidSequence(
      Sequence::fillFrontRearLightingState(COLOR_WHITE_DIM, COLOR_RED_DIM, platform_))},
  };

  current_sequence_ = lighting_sequence_.at(state_);

  // Initialize ROS2 components
  initializePublishers();
  initializeTimers();
  initializeSubscribers();
}

/**
 * @brief Lighting node main loop
 */
void Lighting::spinOnce()
{
  // Update lighting state using latest status messages
  updateState();

  // Determine if user commands should be allowed
  switch (state_)
  {
    case State::Idle:
    case State::Driving:
    case State::ShorePower:
    case State::ShoreAndCharging:
    case State::ShoreAndCharged:
    case State::Charging:
    case State::Charged:
      user_commands_allowed_ = true;
      break;
    case State::LowBattery:
    case State::NeedsReset:
    case State::BatteryFault:
    case State::ShoreFault:
   //case State::PumaFault:
    case State::Stopped:
      user_commands_allowed_ = false;
      break;
  }

  // Change lighting sequence if state has changed
  if (old_state_ != state_)
  {
    current_sequence_ = lighting_sequence_.at(state_);
    current_sequence_.reset();
    old_state_ = state_;
  }
  
  // Get current lighting state from sequence
  lights_msg_ = current_sequence_.getLightsMsg();

  // If user is not commanding lights, update lights
  if (user_timeout_timer_->is_canceled())
  {
    cmd_lights_pub_->publish(lights_msg_);
  }
}

/**
 * @brief Initialize publishers
 */
void Lighting::initializePublishers()
{
  cmd_lights_pub_ = this->create_publisher<clearpath_platform_msgs::msg::Lights>(
    "platform/mcu/_cmd_lights",
    rclcpp::SensorDataQoS());
}

/**
 * @brief Initialize subscribers
 */
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
  power_sub_ = this->create_subscription<clearpath_platform_msgs::msg::Power>(
    "platform/mcu/status/power",
    rclcpp::SensorDataQoS(),
    std::bind(&Lighting::powerCallback, this, std::placeholders::_1));

  // Battery state
  battery_state_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    "platform/bms/state",
    rclcpp::SensorDataQoS(),
    std::bind(&Lighting::batteryStateCallback, this, std::placeholders::_1));
  
  // Stop engaged
  stop_engaged_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "platform/emergency_stop",
    rclcpp::SensorDataQoS(),
    std::bind(&Lighting::stopEngagedCallback, this, std::placeholders::_1));
  
  // Command vel
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "platform/cmd_vel_unstamped",
    rclcpp::SensorDataQoS(),
    std::bind(&Lighting::cmdVelCallback, this, std::placeholders::_1));
}

/**
 * @brief Initialize timers
 */
void Lighting::initializeTimers()
{
  // Start user timeout so that user_timeout_timer_ is not null
  startUserTimeoutTimer();

  // Main loop timer
  lighting_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(LIGHTING_TIMER_TIMEOUT_MS),
    [this]() -> void
    {
      this->spinOnce();
    }
  );
}

/**
 * @brief Start user command timeout timer
 */
void Lighting::startUserTimeoutTimer()
{
  // Reset timer if active
  if (user_timeout_timer_ && !user_timeout_timer_->is_canceled())
  {
    user_timeout_timer_->reset();
  }
  else // Create new timer
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

/**
 * @brief User light command callback
 */
void Lighting::cmdLightsCallback(const clearpath_platform_msgs::msg::Lights::SharedPtr msg)
{
  // Do nothing if user commands are not allowed
  if (!user_commands_allowed_)
  {
    return;
  }

  // Reset user timeout timer
  startUserTimeoutTimer();

  // Publish if allowed
  cmd_lights_pub_->publish(*msg);
}

/**
 * @brief MCU status callback
 */
void Lighting::statusCallback(const clearpath_platform_msgs::msg::Status::SharedPtr msg)
{
  status_msg_ = *msg;
}

/**
 * @brief MCU power status callback
 */
void Lighting::powerCallback(const clearpath_platform_msgs::msg::Power::SharedPtr msg)
{
  power_msg_ = *msg;
}

/**
 * @brief BMS state callback
 */
void Lighting::batteryStateCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  battery_state_msg_ = *msg;
}

/**
 * @brief Emergency stop callback
 */
void Lighting::stopEngagedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  stop_engaged_msg_ = *msg;
}

/**
 * @brief Command velocity callback
 */
void Lighting::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_msg_ = *msg;
}

/**
 * @brief Update state if new state is higher priority
 */
void Lighting::setState(Lighting::State new_state)
{
  if (new_state < state_)
  {
    state_ = new_state;
  }
}

/**
 * @brief Update lighting state based on all status messages
 */
void Lighting::updateState()
{
  state_ = State::Idle;
  // Shore power connected
  if (power_msg_.shore_power_connected == 1)
  {
    // Shore power overvoltage detected on MCU
    if (battery_state_msg_.power_supply_health == sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE)
    {
      setState(State::ShoreFault);
    }
    else
    {
      setState(State::ShorePower);
    }
  }
  else // No shore power
  {
    // Battery health is not good
    if (battery_state_msg_.power_supply_health != sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD)
    {
      setState(State::BatteryFault);
    }
    else if (battery_state_msg_.percentage < 0.2) // Battery low
    {
      setState(State::LowBattery);
    }
  }

  // Charger connected
  if (battery_state_msg_.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING) // || wibotic_charging_msg_.data)
  {
    // Fully charged
    if (battery_state_msg_.percentage == 1.0)
    {
      // Shore power connected
      if (power_msg_.shore_power_connected == 1)
      {
        setState(State::ShoreAndCharged);
      }
      else
      {
        setState(State::Charged);
      }
    }
    else
    {
      // Shore power connected
      if (power_msg_.shore_power_connected == 1)
      {
        setState(State::ShoreAndCharging);
      }
      else
      {
        setState(State::Charging);
      }
    }    
  }
  else if (stop_engaged_msg_.data) // E-Stop
  {
    setState(State::Stopped);
  }
  else if (cmd_vel_msg_.linear.x != 0.0 ||
           cmd_vel_msg_.linear.y != 0.0 ||
           cmd_vel_msg_.angular.z != 0.0) // Robot is driving
  {
    setState(State::Driving);
  }
}
