/**
 *
 *  \file
 *  \brief      Color Class
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

#include <cmath>
#include "clearpath_platform/lighting/color.hpp"
#include <iostream>

using clearpath_lighting::ColorHSV;

ColorHSV::ColorHSV(hsv_t hsv) : hsv_(hsv)
{}

std::vector<ColorHSV> ColorHSV::fade(ColorHSV start, ColorHSV end, uint32_t steps)
{
  std::vector<ColorHSV> fade_vector;

  double h_step, s_step, v_step;

  double a = end.h() - start.h();
//   a += a > 180.0 ? -360.0 : a < -180.0 ? 360.0 : 0.0;
  h_step = a / steps;

  s_step = (end.s() - start.s()) / steps;
  v_step = (end.v() - start.v()) / steps;

  fade_vector.push_back(start);

  for (uint32_t i = 0; i < steps - 2; i++)
  {
    hsv_t hsv = hsv_t(
      fade_vector.back().h() + h_step,
      fade_vector.back().s() + s_step,
      fade_vector.back().v() + v_step);
    fade_vector.push_back(ColorHSV(hsv));
  }

  fade_vector.push_back(end);

  return fade_vector;
}

clearpath_platform_msgs::msg::RGB ColorHSV::getRgbMsg()
{
  clearpath_platform_msgs::msg::RGB rgb;
  int i;
  double f,p,q,t;
  double h, s, v;

  //expand the u8 hue in range 0->255 to 0->359* (there are problems at exactly 360)
  h = hsv_.h;

  h = std::max(0.0, std::min(360.0, h));
  s = std::max(0.0, std::min(100.0, hsv_.s));
  v = std::max(0.0, std::min(100.0, hsv_.v));

  s /= 100.0;
  v /= 100.0;

  if(s == 0) {
    // Achromatic (grey)
    rgb.red = rgb.green = rgb.blue = std::round(v*255.0);
    return rgb;
  }

  h /= 60.0; // sector 0 to 5
  i = std::floor(h);
  f = h - i; // factorial part of h
  p = v * (1 - s);
  q = v * (1 - s * f);
  t = v * (1 - s * (1 - f));
  switch(i) {
    case 0:
        rgb.red = std::round(255.0*v);
        rgb.green = std::round(255.0*t);
        rgb.blue = std::round(255.0*p);
        break;
    case 1:
        rgb.red = std::round(255.0*q);
        rgb.green = std::round(255.0*v);
        rgb.blue = std::round(255.0*p);
        break;
    case 2:
        rgb.red = std::round(255.0*p);
        rgb.green = std::round(255.0*v);
        rgb.blue = std::round(255.0*t);
        break;
    case 3:
        rgb.red = std::round(255.0*p);
        rgb.green = std::round(255.0*q);
        rgb.blue = std::round(255.0*v);
        break;
    case 4:
        rgb.red = std::round(255.0*t);
        rgb.green = std::round(255.0*p);
        rgb.blue = std::round(255.0*v);
        break;
    default: // case 5:
        rgb.red = std::round(255.0*v);
        rgb.green = std::round(255.0*p);
        rgb.blue = std::round(255.0*q);
   }
  return rgb;
}

