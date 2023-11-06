/**
 *
 *  \file
 *  \brief      Pattern Class
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

#include "clearpath_platform/lighting/sequence.hpp"
#include "clearpath_platform/lighting/color.hpp"
#include <cmath>
#include <iostream>

using clearpath_lighting::BlinkSequence;
using clearpath_lighting::PulseSequence;
using clearpath_lighting::Sequence;
using clearpath_lighting::SolidSequence;

clearpath_platform_msgs::msg::Lights Sequence::getLightsMsg()
{
  if (current_state_ >= num_states_)
  {
    current_state_ = 0;
  }

  clearpath_platform_msgs::msg::Lights lights_msg;
  lights_msg.lights.resize(sequence_.size());

  for (uint32_t i = 0; i < sequence_.size(); i++)
  {
    lights_msg.lights.at(i) = sequence_.at(i).at(current_state_).getRgbMsg();
  }

  current_state_++;

  return lights_msg;
}

Sequence::Sequence() : current_state_(0)
{
}

SolidSequence::SolidSequence(const LightingState state)
{
  sequence_.resize(state.size());
  for (uint8_t rgb = 0; rgb < state.size(); rgb++)
  {
    sequence_.at(rgb).push_back(state.at(rgb));
  }
  num_states_ = 1;
}

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
  for (uint32_t i = 0; i < sequence_.at(0).size(); i++)
  {
    std::cout << sequence_.at(0).at(i).h() << std::endl;
  }
}

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

  for (uint32_t i = 0; i < sequence_.at(0).size(); i++)
  {
    uint8_t r, g, b;
    r = sequence_.at(0).at(i).getRgbMsg().red;
    g = sequence_.at(0).at(i).getRgbMsg().green;
    b = sequence_.at(0).at(i).getRgbMsg().blue;
    std::cout << "RGB: [" << static_cast<int>(r) << ", " << static_cast<int>(g) << ", " << static_cast<int>(b) << "]" << std::endl;
  }
}
