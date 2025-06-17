/**
Software License Agreement (BSD)

\file      clearpath_diagnostic_updater.cpp
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

#include "clearpath_diagnostics/clearpath_diagnostic_updater.hpp"

#define UNKNOWN "unknown"

using namespace clearpath_diagnostic_labels;

namespace clearpath
{

/**
 * @brief Construct a new ClearpathDiagnosticUpdater node
 */
ClearpathDiagnosticUpdater::ClearpathDiagnosticUpdater()
: Node(
    "clearpath_diagnostic_updater",
    rclcpp::NodeOptions().allow_undeclared_parameters(true).
    automatically_declare_parameters_from_overrides(true)),
  updater_(this)  // Create the diagnostic updater object
{
  // Get mandatory parameters from the config
  serial_number_ = get_string_param("serial_number", true);
  platform_model_ = get_string_param("platform_model", true);
  ros_distro_ = get_string_param("ros_distro", true);
  latest_apt_firmware_version_ = get_string_param("latest_apt_firmware_version", true);
  installed_apt_firmware_version_ = get_string_param("installed_apt_firmware_version", true);
  RCLCPP_INFO(this->get_logger(), "Diagnostics starting for a %s platform with serial number %s",
              platform_model_.c_str(), serial_number_.c_str());

  // Get optional parameters from the config
  mcu_status_topic_ = get_string_param("mcu_status_topic");
  mcu_status_topic_ = (mcu_status_topic_ == UNKNOWN) ? "platform/mcu/status" : mcu_status_topic_;
  mcu_power_topic_ = get_string_param("mcu_power_topic");
  mcu_power_topic_ = (mcu_power_topic_ == UNKNOWN) ? "platform/mcu/status/power" : mcu_power_topic_;
  bms_state_topic_ = get_string_param("bms_state_topic");
  bms_state_topic_ = (bms_state_topic_ == UNKNOWN) ? "platform/bms/state" : bms_state_topic_;
  stop_status_topic_ = get_string_param("stop_status_topic");
  stop_status_topic_ =
    (stop_status_topic_ == UNKNOWN) ? "platform/mcu/status/stop" : stop_status_topic_;
  estop_topic_ = get_string_param("estop_topic");
  estop_topic_ = (estop_topic_ == UNKNOWN) ? "platform/emergency_stop" : estop_topic_;
  mcu_status_rate_ = get_double_param("mcu_status_rate");
  mcu_status_rate_ = (std::isnan(mcu_status_rate_)) ? 1.0 : mcu_status_rate_;
  mcu_status_tolerance_ = get_double_param("mcu_status_tolerance");
  mcu_status_tolerance_ = (std::isnan(mcu_status_tolerance_)) ? 0.15 : mcu_status_tolerance_;
  mcu_power_rate_ = get_double_param("mcu_power_rate");
  mcu_power_rate_ = (std::isnan(mcu_power_rate_)) ? 10.0 : mcu_power_rate_;
  mcu_power_tolerance_ = get_double_param("mcu_power_tolerance");
  mcu_power_tolerance_ = (std::isnan(mcu_power_tolerance_)) ? 0.15 : mcu_power_tolerance_;
  bms_state_rate_ = get_double_param("bms_state_rate");
  bms_state_rate_ = (std::isnan(bms_state_rate_)) ? 10.0 : bms_state_rate_;
  bms_state_tolerance_ = get_double_param("bms_state_tolerance");
  bms_state_tolerance_ = (std::isnan(bms_state_tolerance_)) ? 0.15 : bms_state_tolerance_;
  stop_status_rate_ = get_double_param("stop_status_rate");
  stop_status_rate_ = (std::isnan(stop_status_rate_)) ? 1.0 : stop_status_rate_;
  stop_status_tolerance_ = get_double_param("stop_status_tolerance");
  stop_status_tolerance_ = (std::isnan(stop_status_tolerance_)) ? 0.15 : stop_status_tolerance_;
  estop_rate_ = get_double_param("estop_rate");
  estop_rate_ = (std::isnan(estop_rate_)) ? 1.0 : estop_rate_;
  estop_tolerance_ = get_double_param("estop_tolerance");
  estop_tolerance_ = (std::isnan(estop_tolerance_)) ? 0.15 : estop_tolerance_;

  // Initialize variables that are populated in callbacks
  mcu_firmware_version_ = UNKNOWN;

  // Set Hardware ID as serial number in diagnostics
  updater_.setHardwareID(serial_number_);

  // MCU status and firmware version if there is an MCU
  if (latest_apt_firmware_version_ == "not_applicable") {
    RCLCPP_INFO(this->get_logger(), "No MCU indicated, MCU diagnostics disabled.");
  } else if (latest_apt_firmware_version_ != "simulated") {
    // Subscribe to MCU Status topics
    sub_mcu_status_ =
      this->create_subscription<clearpath_platform_msgs::msg::Status>(
        mcu_status_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&ClearpathDiagnosticUpdater::mcu_status_callback, this, std::placeholders::_1));

    // Create MCU Frequency Status tracking objects
    mcu_status_freq_status_ = std::make_shared<FrequencyStatus>(
      FrequencyStatusParam(&mcu_status_rate_, &mcu_status_rate_, mcu_status_tolerance_, 10));

    // Add diagnostic tasks for MCU data
    updater_.add("MCU Status", this, &ClearpathDiagnosticUpdater::mcu_status_diagnostic);
    updater_.add("MCU Firmware Version", this, &ClearpathDiagnosticUpdater::firmware_diagnostic);
    RCLCPP_INFO(this->get_logger(), "MCU diagnostics started.");
  }

  // Diagnostics Applicable to all robot platforms irrelevant of which MCU or battery
  // Create subscriptions
  sub_mcu_power_ =
    this->create_subscription<clearpath_platform_msgs::msg::Power>(
      mcu_power_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ClearpathDiagnosticUpdater::mcu_power_callback, this, std::placeholders::_1));
  sub_bms_state_ =
    this->create_subscription<BatteryState>(
      bms_state_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ClearpathDiagnosticUpdater::bms_state_callback, this, std::placeholders::_1));
  sub_stop_status_ =
    this->create_subscription<clearpath_platform_msgs::msg::StopStatus>(
      stop_status_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ClearpathDiagnosticUpdater::stop_status_callback, this, std::placeholders::_1));
  sub_estop_ =
    this->create_subscription<std_msgs::msg::Bool>(
      estop_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ClearpathDiagnosticUpdater::estop_callback, this, std::placeholders::_1));

  // Create Frequency Status tracking objects
  mcu_power_freq_status_ = std::make_shared<FrequencyStatus>(
    FrequencyStatusParam(&mcu_power_rate_, &mcu_power_rate_, mcu_power_tolerance_, 10));
  bms_state_freq_status_ = std::make_shared<FrequencyStatus>(
    FrequencyStatusParam(&bms_state_rate_, &bms_state_rate_, bms_state_tolerance_, 10));
  stop_status_freq_status_ = std::make_shared<FrequencyStatus>(
    FrequencyStatusParam(&stop_status_rate_, &stop_status_rate_, stop_status_tolerance_, 10));
  estop_freq_status_ = std::make_shared<FrequencyStatus>(
    FrequencyStatusParam(&estop_rate_, &estop_rate_, estop_tolerance_, 10));

  // Add diagnostic tasks
  updater_.add("Power Status", this, &ClearpathDiagnosticUpdater::mcu_power_diagnostic);
  updater_.add("Battery Management System", this,
    &ClearpathDiagnosticUpdater::bms_state_diagnostic);
  if (stop_status_rate_ == 0.0) {
    updater_.add("E-stop Status", this, &ClearpathDiagnosticUpdater::estop_diagnostic);
  } else {
    updater_.add("E-stop Status", this, &ClearpathDiagnosticUpdater::stop_status_diagnostic);
  }

  setup_topic_rate_diagnostics();
}

/**
 * @brief Get string parameter from config yaml and optionally log an error if parameter is missing
 */
std::string ClearpathDiagnosticUpdater::get_string_param(std::string param_name, bool mandatory)
{
  try {
    return this->get_parameter(param_name).as_string();
  } catch (const std::exception & e) {
    if (mandatory) {
      RCLCPP_ERROR(this->get_logger(), "Could not retrieve mandatory parameter %s: %s",
                                        param_name.c_str(), e.what());
    }
    return UNKNOWN;
  }
}

/**
 * @brief Get double parameter from config yaml and optionally log an error if parameter is missing
 */
double ClearpathDiagnosticUpdater::get_double_param(std::string param_name, bool mandatory)
{
  try {
    return this->get_parameter(param_name).as_double();
  } catch (const std::exception & e) {
    if (mandatory) {
      RCLCPP_ERROR(this->get_logger(), "Could not retrieve mandatory parameter %s: %s",
                                        param_name.c_str(), e.what());
    }
    return NAN;
  }
}

/**
 * @brief Report the firmware version information to diagnostics
 */
void ClearpathDiagnosticUpdater::firmware_diagnostic(DiagnosticStatusWrapper & stat)
{
  if (latest_apt_firmware_version_ == "not_found") {
    stat.summaryf(DiagnosticStatus::ERROR,
                  "ros-%s-clearpath-firmware package not found. Restart service to re-evaluate.",
                  ros_distro_.c_str());
  } else if (latest_apt_firmware_version_ == UNKNOWN) {
    stat.summaryf(DiagnosticStatus::ERROR,
                  "ros-%s-clearpath-firmware package version not provided in config. Restart service to re-evaluate.",
                  ros_distro_.c_str());
  } else if (mcu_firmware_version_ == UNKNOWN) {
    stat.summary(DiagnosticStatus::ERROR,
                  "No firmware version received from MCU");
  } else if (mcu_firmware_version_ == latest_apt_firmware_version_) {
    stat.summaryf(DiagnosticStatus::OK,
                  "Firmware is up to date (%s)",
                  mcu_firmware_version_.c_str());
  } else if (mcu_firmware_version_ < latest_apt_firmware_version_) {
    stat.summaryf(DiagnosticStatus::WARN,
                  "New firmware available: (%s) -> (%s). Restart service to re-evaluate.",
                  mcu_firmware_version_.c_str(),
                  latest_apt_firmware_version_.c_str());
  } else if (mcu_firmware_version_ > latest_apt_firmware_version_) {
    stat.summaryf(DiagnosticStatus::OK,
                  "Firmware is newer than apt package: (%s) > (%s)",
                  mcu_firmware_version_.c_str(),
                  latest_apt_firmware_version_.c_str());
  } else {
    stat.summaryf(DiagnosticStatus::WARN,
                  "ros-%s-clearpath-firmware package is outdated. Restart service to re-evaluate.",
                  ros_distro_.c_str());
  }
  stat.add("Latest Firmware Version Package", latest_apt_firmware_version_);
  stat.add("Firmware Version Installed on Computer", installed_apt_firmware_version_);
  stat.add("Firmware Version on MCU", mcu_firmware_version_);
}

/**
 * @brief Save data from MCU Status messages
 */
void ClearpathDiagnosticUpdater::mcu_status_callback(
  const clearpath_platform_msgs::msg::Status & msg)
{
  mcu_firmware_version_ = msg.firmware_version;
  mcu_status_msg_ = msg;
  mcu_status_freq_status_->tick();
}

/**
 * @brief Report MCU Status message information to diagnostics
 */
void ClearpathDiagnosticUpdater::mcu_status_diagnostic(DiagnosticStatusWrapper & stat)
{
  mcu_status_freq_status_->run(stat);

  if (stat.level != diagnostic_updater::DiagnosticStatusWrapper::ERROR) {
    // if status messages are being received then add the message details
    stat.add("Firmware Version", mcu_firmware_version_);
    stat.add("Platform Model", mcu_status_msg_.hardware_id);
    stat.add("MCU Uptime (sec)", mcu_status_msg_.mcu_uptime.sec);
    stat.add("Connection Uptime (sec)", mcu_status_msg_.connection_uptime.sec);
  }
}

/**
 * @brief Save data from MCU Power messages
 */
void ClearpathDiagnosticUpdater::mcu_power_callback(const clearpath_platform_msgs::msg::Power & msg)
{
  mcu_power_msg_ = msg;
  mcu_power_freq_status_->tick();
}

/**
 * @brief Report MCU Power message information to diagnostics
 */
void ClearpathDiagnosticUpdater::mcu_power_diagnostic(DiagnosticStatusWrapper & stat)
{
  mcu_power_freq_status_->run(stat);

  if (stat.level != diagnostic_updater::DiagnosticStatusWrapper::ERROR) {
    // if messages are being received then add the message details
    try {
      // check if each datapoint is applicable before displaying it
      if (mcu_power_msg_.shore_power_connected >= 0) {
        stat.add("Shore Power Connected",
          DiagnosticLabels::POWER_STATUS.at(mcu_power_msg_.shore_power_connected));
      }
      if (mcu_power_msg_.battery_connected >= 0) {
        stat.add("Battery Connected",
          DiagnosticLabels::POWER_STATUS.at(mcu_power_msg_.battery_connected));
      }
      if (mcu_power_msg_.power_12v_user_nominal >= 0) {
        stat.add("Power 12V User Nominal",
          DiagnosticLabels::POWER_STATUS.at(mcu_power_msg_.power_12v_user_nominal));
      }
      if (mcu_power_msg_.charger_connected >= 0) {
        stat.add("Charger Connected",
          DiagnosticLabels::POWER_STATUS.at(mcu_power_msg_.charger_connected));
      }
      if (mcu_power_msg_.charging_complete >= 0) {
        stat.add("Charging Complete",
          DiagnosticLabels::POWER_STATUS.at(mcu_power_msg_.charging_complete));
      }
    } catch(const std::out_of_range & e) {
      RCLCPP_ERROR(this->get_logger(),
                  "Unknown MCU Power message status value with no string description: %s",
                  e.what());
    }

    try {
      unsigned int count_v = std::min(mcu_power_msg_.measured_voltages.size(),
                             DiagnosticLabels::MEASURED_VOLTAGES.at(platform_model_).size());
      for (unsigned i = 0; i < count_v; i++) {
        std::string name = "Measured Voltage: " +
          DiagnosticLabels::MEASURED_VOLTAGES.at(platform_model_)[i] + " (V)";
        stat.add(name, mcu_power_msg_.measured_voltages[i]);
      }
      unsigned int count_c = std::min(mcu_power_msg_.measured_currents.size(),
                             DiagnosticLabels::MEASURED_CURRENTS.at(platform_model_).size());
      for (unsigned i = 0; i < count_c; i++) {
        std::string name = "Measured Current: " +
          DiagnosticLabels::MEASURED_CURRENTS.at(platform_model_)[i] + " (A)";
        stat.add(name, mcu_power_msg_.measured_currents[i]);
      }
    } catch(const std::out_of_range & e) {
      RCLCPP_ERROR(this->get_logger(),
                  "No measured voltage or current labels for the given platform: %s", e.what());
    }
  }
}

/**
 * @brief Save data from BMS State / BatteryState messages
 */
void ClearpathDiagnosticUpdater::bms_state_callback(const BatteryState & msg)
{
  bms_state_msg_ = msg;
  bms_state_freq_status_->tick();
}

/**
 * @brief Report BMS State / BatteryState message information to diagnostics
 */
void ClearpathDiagnosticUpdater::bms_state_diagnostic(DiagnosticStatusWrapper & stat)
{
  std::string power_supply_status = "undefined";
  std::string power_supply_health = "undefined";
  std::string power_supply_technology = "undefined";
  try {
    power_supply_status =
      DiagnosticLabels::POWER_SUPPLY_STATUS.at(bms_state_msg_.power_supply_status);
    power_supply_health =
      DiagnosticLabels::POWER_SUPPLY_HEALTH.at(bms_state_msg_.power_supply_health);
    power_supply_technology =
      DiagnosticLabels::POWER_SUPPLY_TECHNOLOGY.at(bms_state_msg_.power_supply_technology);
  } catch(const std::out_of_range & e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Unknown Battery State enum with no string description: %s", e.what());
  }

  bms_state_freq_status_->run(stat);

  if (stat.level != diagnostic_updater::DiagnosticStatusWrapper::ERROR) {
    // if messages are being received then add the message details
    stat.add("Power Supply Status", power_supply_status);
    stat.add("Power Supply Health", power_supply_health);
    stat.add("Power Supply Technology", power_supply_technology);

    stat.add("Voltage (V)", bms_state_msg_.voltage);
    stat.add("Temperature (C)", bms_state_msg_.temperature);
    stat.add("Current (A)", bms_state_msg_.current);
    stat.add("Charge (Ah)", bms_state_msg_.charge);
    stat.add("Capacity (Ah)", bms_state_msg_.capacity);
    stat.add("Charge Percentage", bms_state_msg_.percentage * 100);

    std::string voltages = "";
    for (auto v : bms_state_msg_.cell_voltage) {
      voltages.append(std::to_string(v));
      voltages.append("; ");
    }
    stat.add("Cell Voltages (V)", voltages);

    std::string temperatures = "";
    for (auto t : bms_state_msg_.cell_temperature) {
      temperatures.append(std::to_string(t));
      temperatures.append("; ");
    }
    stat.add("Cell Temperature (C)", temperatures);

    // Diagnostic summaries based on charging activity / level
    if (bms_state_msg_.header.stamp.sec != 0) {
      if (bms_state_msg_.power_supply_status == BatteryState::POWER_SUPPLY_STATUS_CHARGING) {
        stat.mergeSummaryf(DiagnosticStatus::OK,
                            "Battery Charging (%.1f%%)",
                            bms_state_msg_.percentage * 100);
      } else if (bms_state_msg_.percentage >= 0.2) {
        stat.mergeSummaryf(DiagnosticStatus::OK,
                            "Battery level is %.1f%%",
                            bms_state_msg_.percentage * 100);
      } else if (bms_state_msg_.percentage >= 0.1) {
        stat.mergeSummaryf(DiagnosticStatus::WARN,
                            "Low Battery (%.1f%%)",
                            bms_state_msg_.percentage * 100);
      } else {
        stat.mergeSummaryf(DiagnosticStatus::WARN,
                            "Critically Low Battery (%.1f%%)",
                            bms_state_msg_.percentage * 100);
      }

      // Error diagnostic summaries
      if (bms_state_msg_.power_supply_health != BatteryState::POWER_SUPPLY_HEALTH_GOOD) {
        stat.mergeSummaryf(DiagnosticStatus::ERROR,
                            "Power Supply Health: %s", power_supply_health.c_str());
      } else if (bms_state_msg_.power_supply_status == BatteryState::POWER_SUPPLY_STATUS_UNKNOWN) {
        stat.mergeSummary(DiagnosticStatus::ERROR, "Power Supply Status Unknown");
      }
    }
  }
}

/**
 * @brief Save data from Stop Status messages
 */
void ClearpathDiagnosticUpdater::stop_status_callback(
  const clearpath_platform_msgs::msg::StopStatus & msg)
{
  stop_status_msg_ = msg;
  stop_status_freq_status_->tick();
}


/**
 * @brief Save data from E-stop messages
 */
void ClearpathDiagnosticUpdater::estop_callback(
  const std_msgs::msg::Bool & msg)
{
  estop_msg_ = msg;
  estop_freq_status_->tick();
}

/**
 * @brief Report E-stop message information to diagnostics - only used if no MCU
 */
void ClearpathDiagnosticUpdater::estop_diagnostic(DiagnosticStatusWrapper & stat)
{
  estop_freq_status_->run(stat);

  if (stat.level != diagnostic_updater::DiagnosticStatusWrapper::ERROR) {
    // if status messages are being received then add the message details
    stat.add("E-stop Triggered", (estop_msg_.data ? "True" : "False"));

    if (estop_msg_.data) {
      stat.mergeSummary(DiagnosticStatus::WARN, "E-stopped");
    }
  }
}

/**
 * @brief Report E-stop / Stop Status message information to diagnostics
 */
void ClearpathDiagnosticUpdater::stop_status_diagnostic(DiagnosticStatusWrapper & stat)
{
  stop_status_freq_status_->run(stat);

  if (stat.level != diagnostic_updater::DiagnosticStatusWrapper::ERROR) {
    // if status messages are being received then add the message details
    stat.add("E-stop Triggered",
      (estop_msg_.data ? "True" : "False"));
    stat.add("E-stop loop is powered",
      (stop_status_msg_.stop_power_status ? "True" : "False"));
    stat.add("External E-stop has been plugged in",
      (stop_status_msg_.external_stop_present ? "True" : "False"));
    stat.add("Stop loop needs reset",
      (stop_status_msg_.needs_reset ? "True" : "False"));

    if (stop_status_msg_.header.stamp.sec != 0) {
      if (!stop_status_msg_.stop_power_status) {
        stat.mergeSummary(DiagnosticStatus::ERROR, "E-stop loop power error");
      } else if (estop_msg_.data) {
        stat.mergeSummary(DiagnosticStatus::WARN, "E-stopped");
      } else if (stop_status_msg_.needs_reset) {
        stat.mergeSummary(DiagnosticStatus::WARN, "E-stop needs reset");
      }
    }
  }
}

/**
 * @brief Process topics provided in the config yaml and add publish frequency monitoring for each
 */
void ClearpathDiagnosticUpdater::setup_topic_rate_diagnostics()
{
  std::map<std::string, rclcpp::Parameter> topic_map_raw;

  // Get all parameters under the "topics" key in the yaml and store it in map format
  try {
    if (!this->get_parameters("topics", topic_map_raw)) {
      RCLCPP_WARN(this->get_logger(), "No topics found to monitor.");
      return;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "No topics found to monitor.");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Retrieved %zu topic parameters.", topic_map_raw.size());

  // Parse the raw topic map into a map of parameters per topic, stored in another map with
  // the topic as the key
  for (const auto & entry : topic_map_raw) {

    // Identify the topic name and parameter name in the entry key
    auto key = entry.first;  // Contains topic_name.param_name
    auto pos = key.find(".");
    auto topic_name = key.substr(0, pos);
    auto param_name = key.substr(pos + 1);

    // RCLCPP_INFO(this->get_logger(),
    //             "Diagnostics config: Topic is %s, parameter %s = %s.",
    //             topic_name.c_str(), param_name.c_str(), entry.second.value_to_string().c_str());

    // Add the parameter to the topic specific map
    if (auto it = topic_map_.find(topic_name); it != topic_map_.end()) {
      // Topic already exists as a key
      it->second[param_name] = entry.second;
    } else {
      // Topic needs to be added as a new key
      std::map<std::string, rclcpp::Parameter> map;
      map[param_name] = entry.second;
      topic_map_[topic_name] = map;
    }
  }

  // For each topic, create a subscription that monitors the publishing frequency
  for (const auto & topic : topic_map_) {
    auto topic_name = topic.first;
    RCLCPP_INFO(this->get_logger(), "Diagnostics config: Topic is %s", topic_name.c_str());
    auto params_map = topic.second;

    for(const auto & param : params_map) {
      RCLCPP_INFO(this->get_logger(), "  Param: %s = %s",
                  param.first.c_str(), param.second.value_to_string().c_str());
    }

    // Get the message type
    std::string type;
    try {
      type = params_map.at("type").as_string();
    } catch (const std::out_of_range & e) {
      RCLCPP_ERROR(this->get_logger(), "No type provided for %s. This topic will not be monitored",
                   topic_name.c_str());
      continue;
    } catch(const rclcpp::ParameterTypeException & e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Type provided for %s was not a valid string. Value is %s. \
                   This topic will not be monitored",
                   topic_name.c_str(), params_map.at("type").value_to_string().c_str());
      continue;
    }

    // Get the expected publishing rate
    double rate = 0.0;
    try {
      rate = params_map.at("rate").as_double();
    } catch (const std::out_of_range & e) {
      RCLCPP_ERROR(this->get_logger(), "No rate provided for %s. This topic will not be monitored",
                   topic_name.c_str());
      continue;
    } catch(const rclcpp::ParameterTypeException & e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Rate provided for %s was not a valid double. Value is %s. \
                   This topic will not be monitored",
                   topic_name.c_str(), params_map.at("rate").value_to_string().c_str());
      continue;
    }

    /*
     * The section below does not use a generic subscription because the generic subscription was
     * observed to have significantly hgiher CPU usage seemingly related to too short of callbacks
     * and allocating/releasing the memory too quickly with Fast DDS. Standard subscriptions
     * perform more reliably.
     */

    // Create a subscription using the topic name and message type info from the yaml
    if (type == "sensor_msgs/msg/CompressedImage") {
      add_rate_diagnostic<sensor_msgs::msg::CompressedImage>(topic_name, rate);
    } else if (type == "sensor_msgs/msg/Image") {
      add_rate_diagnostic<sensor_msgs::msg::Image>(topic_name, rate);
    } else if (type == "sensor_msgs/msg/Imu") {
      add_rate_diagnostic<sensor_msgs::msg::Imu>(topic_name, rate);
    } else if (type == "sensor_msgs/msg/LaserScan") {
      add_rate_diagnostic<sensor_msgs::msg::LaserScan>(topic_name, rate);
    } else if (type == "sensor_msgs/msg/MagneticField") {
      add_rate_diagnostic<sensor_msgs::msg::MagneticField>(topic_name, rate);
    } else if (type == "sensor_msgs/msg/NavSatFix") {
      add_rate_diagnostic<sensor_msgs::msg::NavSatFix>(topic_name, rate);
    } else if (type == "sensor_msgs/msg/PointCloud2") {
      add_rate_diagnostic<sensor_msgs::msg::PointCloud2>(topic_name, rate);
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Type \"%s\" of topic \"%s\" is not supported",
                   type.c_str(),
                   topic_name.c_str());
      continue;
    }

    RCLCPP_INFO(this->get_logger(), "Created subscription for %s", topic_name.c_str());
  }
}

/**
 * @brief Template to add rate diagnostics for a given topic and rate
 */
template<class MsgType> void ClearpathDiagnosticUpdater::add_rate_diagnostic(
  const std::string topic_name, const double rate)
{
  // Store the rate so that it can be accessed via a pointer and is not deleted
  rates_.push_back(rate);

  // Create the diagnostic task object that handles calculating and publishing rate statistics
  auto topic_diagnostic =
    std::make_shared<diagnostic_updater::HeaderlessTopicDiagnostic>(
      topic_name,
      updater_,
      FrequencyStatusParam(&rates_.back(), &rates_.back(), 0.1, 5));

  // Store the diagnostic task object so that it can be accessed via a pointer and is not deleted
  topic_diagnostics_.push_back(topic_diagnostic);

  auto sub = this->create_subscription<MsgType>(
    topic_name,
    rclcpp::SensorDataQoS(),
    [this, topic_diagnostic]
    ([[maybe_unused]] const MsgType & msg) {
      topic_diagnostic->tick();
    });
  subscriptions_.push_back(std::static_pointer_cast<void>(sub));
}
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<clearpath::ClearpathDiagnosticUpdater>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
