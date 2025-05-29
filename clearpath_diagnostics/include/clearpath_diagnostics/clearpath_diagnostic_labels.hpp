/**
Software License Agreement (BSD)

\file      clearpath_diagnostic_labels.hpp
\authors   Hilary Luo <hluo@clearpathrobotics.com>
\copyright Copyright (c) 2025, Clearpath Robotics, Inc., All rights reserved.

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


#ifndef CLEARPATH_DIAGNOSTIC_LABELS_HPP
#define CLEARPATH_DIAGNOSTIC_LABELS_HPP

#include <map>
#include <vector>

#include "sensor_msgs/msg/battery_state.hpp"

using BatteryState = sensor_msgs::msg::BatteryState;

namespace clearpath_diagnostic_labels
{

class DiagnosticLabels {

public:
  //--------------------------------------------------------
  // Robot Platforms
  //--------------------------------------------------------

  // Dingo D V1
  inline static const std::string DD100 = "dd100";
  // Dingo O V1
  inline static const std::string DO100 = "do100";
  // Dingo D V1.5
  inline static const std::string DD150 = "dd150";
  // Dingo D V1.5
  inline static const std::string DO150 = "do150";
  // Jackal V1
  inline static const std::string J100 = "j100";
  // Husky V2
  inline static const std::string A200 = "a200";
  // Husky V3
  inline static const std::string A300 = "a300";
  // Ridgeback V1
  inline static const std::string R100 = "r100";
  // Warthog V2
  inline static const std::string W200 = "w200";
  // Genric Robot
  inline static const std::string GENERIC = "generic";

  //--------------------------------------------------------
  // Labels for clearpath_platform_msgs::msg::Power
  //--------------------------------------------------------

  // int8 statuses
  inline static const std::map<int8_t, std::string> POWER_STATUS = {
    {-1, "Not Applicable"},
    {0, "False"},
    {1, "True"}
  };

  // Common Core Measured Voltage Labels
  inline static const std::vector<std::string> CC01_MEASURED_VOLTAGES = {
    "Battery",
    "User Battery",
    "User 24V",
    "User 12V",
    "System 12V",
    "Expansion",
    "Breakout 24V Aux",
    "Breakout 12V Aux",
    "Breakout User 12VA",
    "Breakout Lynx1",
    "Breakout Lynx2",
    "Breakout Lynx3",
    "Breakout Lynx4"
  };

  // Common Core Measured Currents Labels
  inline static const std::vector<std::string> CC01_MEASURED_CURRENTS = {
    "Battery",
    "User Battery",
    "Aux",
    "System 12V",
    "24V",
    "12V",
    "Breakout User 24V",
    "Breakout User 12VA",
    "Breakout User 12VB",
    "Breakout Lynx1",
    "Breakout Lynx2",
    "Breakout Lynx3",
    "Breakout Lynx4"
  };

  inline static const std::map<std::string, std::vector<std::string>> MEASURED_VOLTAGES = {
    // Order and count of labels must match the Power message definition
    {DD100, {"Battery", "5V Rail", "12V Rail"}},
    {DO100, {"Battery", "5V Rail", "12V Rail"}},
    {DD150, {"Battery", "5V Rail", "12V Rail"}},
    {DO150, {"Battery", "5V Rail", "12V Rail"}},
    {J100, {"Battery", "5V Rail", "12V Rail"}},
    {A200, {"Battery", "Left Driver Voltage", "Right Driver Voltage"}},
    {A300, CC01_MEASURED_VOLTAGES},
    {R100, {"Battery", "5V Rail", "12V Rail", "Inverter", "Front Axle", "Rear Axle", "Light"}},
    {W200, {"Battery", "12V Rail", "24V Rail", "48V Rail"}},
  };

  inline static const std::map<std::string, std::vector<std::string>> MEASURED_CURRENTS = {
    // Order and count of labels must match the Power message definition
    {DD100, {"Total", "Computer"}},
    {DO100, {"Total", "Computer"}},
    {DD150, {"Total", "Computer"}},
    {DO150, {"Total", "Computer"}},
    {J100, {"Total", "Computer", "Drive", "User"}},
    {A200, {"MCU and User Port", "Left Driver", "Right Driver", "Currents Size"}},
    {A300, CC01_MEASURED_CURRENTS},
    {R100, {"Total"}},
    {W200, {"Total", "Computer", "12V", "24V"}},
  };

  //--------------------------------------------------------
  // Labels for sensor_msgs::msg::BatteryState
  //--------------------------------------------------------

  inline static const std::map<uint8_t, std::string> POWER_SUPPLY_HEALTH = {
    {BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN, "Unknown"},
    {BatteryState::POWER_SUPPLY_HEALTH_GOOD, "Good"},
    {BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT, "Overheat"},
    {BatteryState::POWER_SUPPLY_HEALTH_DEAD, "Dead"},
    {BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE, "Overvoltage"},
    {BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE, "Unspecified Failure"},
    {BatteryState::POWER_SUPPLY_HEALTH_COLD, "Cold"},
    {BatteryState::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE, "Watchdog Timer Expired"},
    {BatteryState::POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE, "Safety Timer Expired"},
  };

  inline static const std::map<uint8_t, std::string> POWER_SUPPLY_STATUS = {
    {BatteryState::POWER_SUPPLY_STATUS_UNKNOWN, "Unknown"},
    {BatteryState::POWER_SUPPLY_STATUS_CHARGING, "Charging"},
    {BatteryState::POWER_SUPPLY_STATUS_DISCHARGING, "Discharging"},
    {BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING, "Not Charging"},
    {BatteryState::POWER_SUPPLY_STATUS_FULL, "Full"},
  };

  inline static const std::map<uint8_t, std::string> POWER_SUPPLY_TECHNOLOGY = {
    {BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN, "Unknown"},
    {BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH, "NIMH"},
    {BatteryState::POWER_SUPPLY_TECHNOLOGY_LION, "LION"},
    {BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO, "LIPO"},
    {BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE, "LIFE"},
    {BatteryState::POWER_SUPPLY_TECHNOLOGY_NICD, "NICD"},
    {BatteryState::POWER_SUPPLY_TECHNOLOGY_LIMN, "LIMN"},
    {BatteryState::POWER_SUPPLY_TECHNOLOGY_TERNARY, "Ternary"},
    {BatteryState::POWER_SUPPLY_TECHNOLOGY_VRLA, "VRLA"},
  };

};
}

#endif  // CLEARPATH_DIAGNOSTIC_LABELS_HPP
