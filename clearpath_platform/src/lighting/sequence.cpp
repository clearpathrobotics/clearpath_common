/**
 *
 *  \file
 *  \brief      Sequence Classes
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

#include <cmath>

#include "clearpath_platform/lighting/sequence.hpp"
#include "clearpath_platform/lighting/color.hpp"

using clearpath_lighting::BlinkSequence;
using clearpath_lighting::PulseSequence;
using clearpath_lighting::Sequence;
using clearpath_lighting::SolidSequence;
using clearpath_lighting::LightingState;
using clearpath_lighting::Platform;

/**
 * @brief Get Lights message at current state in the sequence
 * Increment current state for the next call
 */
clearpath_platform_msgs::msg::Lights Sequence::getLightsMsg()
{
  // Reset to initial state if we are at the end of the sequence
  if (current_state_ >= num_states_)
  {
    current_state_ = 0;
  }

  // Fill Lights message
  clearpath_platform_msgs::msg::Lights lights_msg;
  lights_msg.lights.resize(sequence_.size());

  for (uint32_t i = 0; i < sequence_.size(); i++)
  {
    lights_msg.lights.at(i) = sequence_.at(i).at(current_state_).getRgbMsg();
  }

  current_state_++;

  return lights_msg;
}

/**
 * @brief Default Sequence constructor
 */
Sequence::Sequence() : current_state_(0)
{}

/**
 * @brief Reset the sequence
 */
void Sequence::reset()
{
  current_state_ = 0;
}

/**
 * @brief Fill all lights with a color
 */
LightingState Sequence::fillLightingState(ColorHSV color, Platform platform)
{
  return LightingState(PlatformNumLights.at(platform), color);
}

/**
 * @brief Fill front and rear lights with separate colors
 */
LightingState Sequence::fillFrontRearLightingState(ColorHSV front_color, ColorHSV rear_color, Platform platform)
{
  LightingState lighting_state(PlatformNumLights.at(platform), COLOR_BLACK);
  switch (platform)
  {
    case Platform::DD100:
    case Platform::DO100:
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D100_LIGHTS_FRONT_LEFT) = front_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D100_LIGHTS_FRONT_RIGHT) = front_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D100_LIGHTS_REAR_LEFT) = rear_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D100_LIGHTS_REAR_RIGHT) = rear_color;
      break;

    case Platform::DD150:
    case Platform::DO150:
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D150_LIGHTS_FRONT_LEFT) = front_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D150_LIGHTS_FRONT_RIGHT) = front_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D150_LIGHTS_REAR_LEFT) = rear_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D150_LIGHTS_REAR_RIGHT) = rear_color;
      break;
    
    case Platform::W200:
      lighting_state.at(clearpath_platform_msgs::msg::Lights::W200_LIGHTS_FRONT_LEFT) = front_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::W200_LIGHTS_FRONT_RIGHT) = front_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::W200_LIGHTS_REAR_LEFT) = rear_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::W200_LIGHTS_REAR_RIGHT) = rear_color;
      break;
    
    case Platform::R100:
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_FRONT_PORT_UPPER) = front_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_FRONT_PORT_LOWER) = front_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_FRONT_STARBOARD_UPPER) = front_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_FRONT_STARBOARD_LOWER) = front_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_REAR_PORT_UPPER) = rear_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_REAR_PORT_LOWER) = rear_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_REAR_STARBOARD_UPPER) = rear_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_REAR_STARBOARD_LOWER) = rear_color;
      break;
  }
  return lighting_state;
}

/**
 * @brief Fill left and right lights with separate colors
 */
LightingState Sequence::fillLeftRightLightingState(ColorHSV left_color, ColorHSV right_color, Platform platform)
{
  LightingState lighting_state(PlatformNumLights.at(platform), COLOR_BLACK);
  switch (platform)
  {
    case Platform::DD100:
    case Platform::DO100:
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D100_LIGHTS_FRONT_LEFT) = left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D100_LIGHTS_REAR_LEFT) = left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D100_LIGHTS_FRONT_RIGHT) = right_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D100_LIGHTS_REAR_RIGHT) = right_color;
      break;

    case Platform::DD150:
    case Platform::DO150:
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D150_LIGHTS_FRONT_LEFT) = left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D150_LIGHTS_REAR_LEFT) = left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D150_LIGHTS_FRONT_RIGHT) = right_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D150_LIGHTS_REAR_RIGHT) = right_color;
      break;
    
    case Platform::W200:
      lighting_state.at(clearpath_platform_msgs::msg::Lights::W200_LIGHTS_FRONT_LEFT) = left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::W200_LIGHTS_REAR_LEFT) = left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::W200_LIGHTS_FRONT_RIGHT) = right_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::W200_LIGHTS_REAR_RIGHT) = right_color;
      break;
    
    case Platform::R100:
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_FRONT_PORT_UPPER) = left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_FRONT_PORT_LOWER) = left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_REAR_PORT_UPPER) = left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_REAR_PORT_LOWER) = left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_FRONT_STARBOARD_UPPER) = right_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_FRONT_STARBOARD_LOWER) = right_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_REAR_STARBOARD_UPPER) = right_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_REAR_STARBOARD_LOWER) = right_color;
      break;
  }
  return lighting_state;
}

/**
 * @brief Fill opposite corner lights with separate colors
 */
LightingState Sequence::fillOppositeCornerLightingState(ColorHSV front_left_color, ColorHSV front_right_color, Platform platform)
{
  LightingState lighting_state(PlatformNumLights.at(platform), COLOR_BLACK);
  switch (platform)
  {
    case Platform::DD100:
    case Platform::DO100:
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D100_LIGHTS_FRONT_LEFT) = front_left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D100_LIGHTS_REAR_RIGHT) = front_left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D100_LIGHTS_FRONT_RIGHT) = front_right_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D100_LIGHTS_REAR_LEFT) = front_right_color;
      break;

    case Platform::DD150:
    case Platform::DO150:
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D150_LIGHTS_FRONT_LEFT) = front_left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D150_LIGHTS_REAR_RIGHT) = front_left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D150_LIGHTS_FRONT_RIGHT) = front_right_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::D150_LIGHTS_REAR_LEFT) = front_right_color;
      break;
    
    case Platform::W200:
      lighting_state.at(clearpath_platform_msgs::msg::Lights::W200_LIGHTS_FRONT_LEFT) = front_left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::W200_LIGHTS_REAR_RIGHT) = front_left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::W200_LIGHTS_FRONT_RIGHT) = front_right_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::W200_LIGHTS_REAR_LEFT) = front_right_color;
      break;
    
    case Platform::R100:
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_FRONT_PORT_UPPER) = front_left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_FRONT_PORT_LOWER) = front_left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_REAR_STARBOARD_UPPER) = front_left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_REAR_STARBOARD_LOWER) = front_left_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_FRONT_STARBOARD_UPPER) = front_right_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_FRONT_STARBOARD_LOWER) = front_right_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_REAR_PORT_UPPER) = front_right_color;
      lighting_state.at(clearpath_platform_msgs::msg::Lights::R100_LIGHTS_REAR_PORT_LOWER) = front_right_color;
      break;
  }
  return lighting_state;
}

/**
 * @brief Solid sequence constructor
 * @param state: Lighting state for solid sequence
 */
SolidSequence::SolidSequence(const LightingState state)
{
  sequence_.resize(state.size());
  for (uint8_t rgb = 0; rgb < state.size(); rgb++)
  {
    sequence_.at(rgb).push_back(state.at(rgb));
  }
  num_states_ = 1;
}

/**
 * @brief Blink sequence constructor
 * @param first_state: Lighting state for "on" time
 * @param second_state: Lighting state for "off" time
 * @param steps: Number of steps in the sequence
 * @param duty_cycle: Percentage of sequence that is in the "on" state. [0.0 ... 1.0]
 */
BlinkSequence::BlinkSequence(const LightingState first_state,
                             const LightingState second_state,
                             uint32_t steps,
                             double duty_cycle)
{
  sequence_.resize(first_state.size());
  if (steps <= 1)
  {
    for (uint8_t i = 0; i < first_state.size(); i++)
    {
      sequence_.at(i).push_back(first_state.at(i));
    }
  }
  else
  {
    for (uint8_t i = 0; i < first_state.size(); i++)
    {
      for (auto j = 0; j < steps * duty_cycle; j++)
      {
        sequence_.at(i).push_back(first_state.at(i));
      }
      for (auto j = 0; j < steps - steps * duty_cycle; j++)
      {
        sequence_.at(i).push_back(second_state.at(i));
      }
    }
  }
  num_states_ = steps;
}

/**
 * @brief Pulse sequence constructor
 * @param first_state: Starting lighting state
 * @param last_state: Ending lighting state
 * @param steps: Number of steps in the sequence
 */
PulseSequence::PulseSequence(const LightingState first_state,
                             const LightingState last_state,
                             uint32_t steps)
{
  sequence_.resize(first_state.size());
  num_states_ = steps;

  // Fade from first to last state, then back to first
  for (uint32_t i = 0; i < first_state.size(); i++)
  {
    sequence_.at(i) = ColorHSV::fade(first_state.at(i), last_state.at(i), steps / 2);
    std::vector<ColorHSV> reverse = sequence_.at(i);
    std::reverse(reverse.begin(), reverse.end());
    sequence_.at(i).insert(std::end(sequence_.at(i)), std::begin(reverse), std::end(reverse));
  }
}
