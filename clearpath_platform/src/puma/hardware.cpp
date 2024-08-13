/**
Software License Agreement (BSD)

\file      hardware.cpp
\authors   Luis Camero <lcamero@clearpathrobotics.com>
\copyright Copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of Clearpath Robotics nor the names of its contributors
  may be used to endorse or promote products derived from this software
  without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/
#include "clearpath_platform/puma/hardware.hpp"

namespace clearpath_platform
{

/**
 * @brief Initialize hardware interface object
*/
hardware_interface::CallbackReturn PumaHardware::initHardwareInterface()
{
  node_ = std::make_shared<PumaHardwareInterface>("puma_hardware_interface");

  if (node_ == nullptr)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Write commands to the hardware
*/
void PumaHardware::writeCommandsToHardware()
{
  sensor_msgs::msg::JointState joint_state;

  for (const auto &it : wheel_joints_)
  {
    joint_state.name.push_back(it.first);
    double speed = hw_commands_[it.second];
    if (std::abs(speed) < MINIMUM_VELOCITY)
    {
      speed = 0.0;
    }
    joint_state.velocity.push_back(speed);
  }
  node_->drive_command(joint_state);
  return;
}

/**
 * @brief Pull latest speed and travel measurements from MCU,
 * and store in joint structure for ROS controls
*/
void PumaHardware::updateJointsFromHardware()
{
  rclcpp::spin_some(node_);

  if (node_->has_new_feedback())
  {

    auto msg = node_->get_feedback();

    for (auto& puma : msg.drivers_feedback)
    {
      hw_states_velocity_[wheel_joints_[puma.device_name]] = puma.speed;
      hw_states_position_[wheel_joints_[puma.device_name]] = puma.travel; // * 0.049;
    }
  }
}

/**
 * @brief Get hardware information from robot description
*/
hardware_interface::CallbackReturn PumaHardware::getHardwareInfo(const hardware_interface::HardwareInfo & info)
{
  // Get info from URDF
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_name_ = info_.name;
  num_joints_ = info_.joints.size();

  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Name: %s", hw_name_.c_str());

  // Check for valid number of joints
  if (num_joints_ != DIFF_DRIVE_FOUR_JOINTS && num_joints_ != DIFF_DRIVE_TWO_JOINTS)
  {
    RCLCPP_ERROR(rclcpp::get_logger(hw_name_), "Invalid number of joints %u", num_joints_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Number of Joints %u", num_joints_);

  hw_states_position_.resize(num_joints_);
  hw_states_position_offset_.resize(num_joints_);
  hw_states_velocity_.resize(num_joints_);
  hw_commands_.resize(num_joints_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Check command interfaces are valid
*/
hardware_interface::CallbackReturn PumaHardware::validateJoints()
{
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // PumaHardware has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Initialization
*/
hardware_interface::CallbackReturn PumaHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  hardware_interface::CallbackReturn ret;
  // Get Hardware name and joints
  ret = getHardwareInfo(info);

  if (ret != hardware_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Validate joints
  ret = validateJoints();

  if (ret != hardware_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Initialize hardware interface
  return initHardwareInterface();
}

/**
 * @brief Map state interfaces
*/
std::vector<hardware_interface::StateInterface> PumaHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < num_joints_; i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }

  return state_interfaces;
}

/**
 * @brief Map command interfaces
*/
std::vector<hardware_interface::CommandInterface> PumaHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto i = 0u; i < num_joints_; i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));

    // Map wheel joint name to index
    wheel_joints_[info_.joints[i].name] = i;
  }

  return command_interfaces;
}

/**
 * @brief Activate
*/
hardware_interface::CallbackReturn PumaHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Starting ...please wait...");

  // set some default values
  for (auto i = 0u; i < hw_states_position_.size(); i++)
  {
    if (std::isnan(hw_states_position_[i]))
    {
      hw_states_position_[i] = 0;
      hw_states_position_offset_[i] = 0;
      hw_states_velocity_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "System Successfully started!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Deactivate
*/
hardware_interface::CallbackReturn PumaHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Stopping ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "System successfully stopped!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Read joint feedback from hardware interface
*/
hardware_interface::return_type PumaHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Reading from hardware");

  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Duration %f", period.seconds());

  updateJointsFromHardware();

  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Joints successfully read!");

  return hardware_interface::return_type::OK;
}

/**
 * @brief Write joint command to hardware interface
*/
hardware_interface::return_type PumaHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Writing to hardware");

  writeCommandsToHardware();

  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

} // namespace clearpath_platform

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  clearpath_platform::PumaHardware, hardware_interface::SystemInterface)
