/**
 *
 *  \file
 *  \brief      Class representing diff drive hardware
 *  \author     Roni Kreinin <rkreinin@clearpathrobotics.com>
 *  \author     Tony Baltovski <tbaltovski@clearpathrobotics.com>
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

#include "clearpath_platform/diff_drive/hardware.hpp"
#include "clearpath_platform_msgs/msg/feedback.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace clearpath_platform
{

static const std::string LEFT_CMD_JOINT_NAME = "front_left_wheel_joint";
static const std::string RIGHT_CMD_JOINT_NAME = "front_right_wheel_joint";
static const std::string LEFT_ALT_JOINT_NAME = "rear_left_wheel_joint";
static const std::string RIGHT_ALT_JOINT_NAME = "rear_right_wheel_joint";

/**
 * @brief Write commanded velocities to the MCU
 * 
 */
void DiffDriveHardware::writeCommandsToHardware()
{
  double diff_speed_left = hw_commands_[wheel_joints_[LEFT_CMD_JOINT_NAME]];
  double diff_speed_right = hw_commands_[wheel_joints_[RIGHT_CMD_JOINT_NAME]];

  if (std::abs(diff_speed_left) < 0.01 && std::abs(diff_speed_right) < 0.01) {
    diff_speed_left = diff_speed_right = 0.0;
  }

  node_->drive_command(
    static_cast<float>(diff_speed_left),
    static_cast<float>(diff_speed_right),
    clearpath_platform_msgs::msg::Drive::MODE_VELOCITY);
}

/**
 * @brief Pull latest speed and travel measurements from MCU, 
 * and store in joint structure for ros_control
 * 
 */
void DiffDriveHardware::updateJointsFromHardware()
{
  rclcpp::spin_some(node_);
  clearpath_platform_msgs::msg::Feedback msg = node_->get_feedback();
  RCLCPP_DEBUG(
    rclcpp::get_logger(hw_name_),
    "Received linear distance information (L: %f, R: %f)",
    msg.drivers[0].measured_travel, msg.drivers[1].measured_travel);

  auto side = clearpath_platform_msgs::msg::Drive::LEFT;
  for (auto i = 0u; i < hw_states_position_.size(); i++) {
    if (num_joints_ == DIFF_DRIVE_TWO_JOINTS)
    {
      if (i == wheel_joints_[RIGHT_CMD_JOINT_NAME]){
        side = clearpath_platform_msgs::msg::Drive::RIGHT;
      }
      else {
        side = clearpath_platform_msgs::msg::Drive::LEFT;
      }
    }
    else if (num_joints_ == DIFF_DRIVE_FOUR_JOINTS)
    {
      if (i == wheel_joints_[RIGHT_CMD_JOINT_NAME] || i == wheel_joints_[RIGHT_ALT_JOINT_NAME]){
        side = clearpath_platform_msgs::msg::Drive::RIGHT;
      }
      else {
        side = clearpath_platform_msgs::msg::Drive::LEFT;
      }
    }

    double delta = msg.drivers[side].measured_travel -
      hw_states_position_[i] - hw_states_position_offset_[i];

    // detect suspiciously large readings, possibly from encoder rollover
    if (std::abs(delta) < 1.0f) {
      hw_states_position_[i] += delta;
    } else {
      // suspicious! drop this measurement and update the offset for subsequent readings
      hw_states_position_offset_[i] += delta;
      RCLCPP_WARN(
        rclcpp::get_logger(hw_name_), "Dropping overflow measurement from encoder");
    }

    // Velocities
    hw_states_velocity_[i] = msg.drivers[side].measured_velocity;
  }
}

hardware_interface::CallbackReturn DiffDriveHardware::getHardwareInfo(const hardware_interface::HardwareInfo & info)
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

hardware_interface::CallbackReturn DiffDriveHardware::validateJoints()
{ 
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffDriveHardware has exactly two states and one command interface on each joint
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

hardware_interface::CallbackReturn DiffDriveHardware::initHardwareInterface()
{
  node_ = std::make_shared<DiffDriveHardwareInterface>("diff_drive_hardware_interface");

  if (node_ == nullptr)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveHardware::on_init(const hardware_interface::HardwareInfo & info)
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

std::vector<hardware_interface::StateInterface> DiffDriveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < num_joints_; i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto i = 0u; i < num_joints_; i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));

    // Map wheel joint name to index
    wheel_joints_[info_.joints[i].name] = i;
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Starting ...please wait...");

  // set some default values
  for (auto i = 0u; i < hw_states_position_.size(); i++) {
    if (std::isnan(hw_states_position_[i])) {
      hw_states_position_[i] = 0;
      hw_states_position_offset_[i] = 0;
      hw_states_velocity_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "System Successfully started!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Stopping ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "System successfully stopped!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Reading from hardware");

  updateJointsFromHardware();

  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Joints successfully read!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Writing to hardware");

  writeCommandsToHardware();

  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

}  // namespace clearpath_platform
