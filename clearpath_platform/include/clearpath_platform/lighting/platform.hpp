/**
 *
 *  \file
 *  \brief      Platform Header
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
 * Adapted from https://gist.github.com/borgel/d9a8bc11aeb5e0005d8320026c46f6f7
 */

#ifndef CLEARPATH_PLATFORM__LIGHTING__PLATFORM_HPP_
#define CLEARPATH_PLATFORM__LIGHTING__PLATFORM_HPP_

#include <map>
#include <stdint.h>

namespace clearpath_lighting
{

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
  {"dd100", Platform::DD100},
  {"do100", Platform::DO100},
  {"dd150", Platform::DD150},
  {"do150", Platform::DO150},
  {"r100", Platform::R100},
  {"w200", Platform::W200},
};

static std::map<Platform, int> PlatformNumLights
{
  {Platform::DD100, 4},
  {Platform::DO100, 4},
  {Platform::DD150, 4},
  {Platform::DO150, 4},
  {Platform::R100, 8},
  {Platform::W200, 4},
};

}

#endif  // CLEARPATH_PLATFORM__LIGHTING__PLATFORM_HPP_
