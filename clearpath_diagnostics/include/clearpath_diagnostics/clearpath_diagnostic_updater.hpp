/**
Software License Agreement (BSD)

\file      clearpath_diagnostic_updater.hpp
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

#ifndef CLEARPATH_DIAGNOSTIC_UPDATER_HPP
#define CLEARPATH_DIAGNOSTIC_UPDATER_HPP

#include <map>
#include <list>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "diagnostic_updater/update_functions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"

#include "clearpath_diagnostics/clearpath_diagnostic_labels.hpp"
#include "clearpath_platform_msgs/msg/power.hpp"
#include "clearpath_platform_msgs/msg/status.hpp"
#include "clearpath_platform_msgs/msg/stop_status.hpp"

namespace clearpath
{

class ClearpathDiagnosticUpdater : public rclcpp::Node
{
public:
  ClearpathDiagnosticUpdater();

private:
  using BatteryState = sensor_msgs::msg::BatteryState;
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
  using DiagnosticStatusWrapper = diagnostic_updater::DiagnosticStatusWrapper;
  using FrequencyStatus = diagnostic_updater::FrequencyStatus;
  using FrequencyStatusParam = diagnostic_updater::FrequencyStatusParam;

  // Callbacks
  void mcu_status_callback(const clearpath_platform_msgs::msg::Status & msg);
  void mcu_power_callback(const clearpath_platform_msgs::msg::Power & msg);
  void bms_state_callback(const BatteryState & msg);
  void stop_status_callback(const clearpath_platform_msgs::msg::StopStatus & msg);
  void estop_callback(const std_msgs::msg::Bool & msg);

  // Diagnostic Tasks
  void firmware_diagnostic(DiagnosticStatusWrapper & stat);
  void mcu_status_diagnostic(DiagnosticStatusWrapper & stat);
  void mcu_power_diagnostic(DiagnosticStatusWrapper & stat);
  void bms_state_diagnostic(DiagnosticStatusWrapper & stat);
  void stop_status_diagnostic(DiagnosticStatusWrapper & stat);
  void estop_diagnostic(DiagnosticStatusWrapper & stat);

  // Get parameters from config
  std::string get_string_param(std::string param_name, bool mandatory = false);
  double get_double_param(std::string param_name, bool mandatory = false);

  void setup_topic_rate_diagnostics();
  template<class MsgType> void add_rate_diagnostic(const std::string topic_name, const double rate);

  // Parameters from config
  std::string serial_number_;
  std::string platform_model_;
  std::string namespace_;
  std::string ros_distro_;  // Specifically the ros distro used for the firmware apt package check
  std::string latest_apt_firmware_version_;
  std::string installed_apt_firmware_version_;
  std::map<std::string, std::map<std::string, rclcpp::Parameter>> topic_map_;

  // Topic names and rates
  std::string mcu_status_topic_;
  std::string mcu_power_topic_;
  std::string bms_state_topic_;
  std::string stop_status_topic_;
  std::string estop_topic_;
  double mcu_status_rate_;
  double mcu_status_tolerance_;
  double mcu_power_rate_;
  double mcu_power_tolerance_;
  double bms_state_rate_;
  double bms_state_tolerance_;
  double stop_status_rate_;
  double stop_status_tolerance_;
  double estop_rate_;
  double estop_tolerance_;

  // Message Data
  std::string mcu_firmware_version_;
  clearpath_platform_msgs::msg::Status mcu_status_msg_;
  clearpath_platform_msgs::msg::Power mcu_power_msg_;
  BatteryState bms_state_msg_;
  clearpath_platform_msgs::msg::StopStatus stop_status_msg_;
  std_msgs::msg::Bool estop_msg_;

  // Frequency statuses
  std::shared_ptr<FrequencyStatus> mcu_status_freq_status_;
  std::shared_ptr<FrequencyStatus> mcu_power_freq_status_;
  std::shared_ptr<FrequencyStatus> bms_state_freq_status_;
  std::shared_ptr<FrequencyStatus> stop_status_freq_status_;
  std::shared_ptr<FrequencyStatus> estop_freq_status_;

  // Subscriptions
  rclcpp::Subscription<clearpath_platform_msgs::msg::Status>::SharedPtr sub_mcu_status_;
  rclcpp::Subscription<clearpath_platform_msgs::msg::Power>::SharedPtr sub_mcu_power_;
  rclcpp::Subscription<BatteryState>::SharedPtr sub_bms_state_;
  rclcpp::Subscription<clearpath_platform_msgs::msg::StopStatus>::SharedPtr sub_stop_status_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;

  // Lists to ensure all variables relating to the rate monitoring persist until spin
  std::list<double> rates_;
  std::list<std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic>> topic_diagnostics_;
  std::list<std::shared_ptr<void>> subscriptions_;

};

}

#endif  // CLEARPATH_DIAGNOSTIC_UPDATER_HPP
