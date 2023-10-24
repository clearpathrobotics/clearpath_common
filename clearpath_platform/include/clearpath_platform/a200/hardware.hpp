#ifndef CLEARPATH_PLATFORM__A200_HARDWARE_HPP_
#define CLEARPATH_PLATFORM__A200_HARDWARE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"
#include "rclcpp/macros.hpp"

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "clearpath_platform/a200/horizon_legacy/horizon_legacy_wrapper.h"
#include "clearpath_platform/a200/status.hpp"

#include "clearpath_platform_msgs/msg/power.hpp"
#include "clearpath_platform_msgs/msg/status.hpp"
#include "clearpath_platform_msgs/msg/stop_status.hpp"

using namespace std::chrono_literals;

namespace clearpath_platform
{

class A200Hardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(A200Hardware)

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void resetTravelOffset();
  double linearToAngular(const double &travel) const;
  double angularToLinear(const double &angle) const;
  void writeCommandsToHardware();
  void limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right);
  void updateJointsFromHardware();
  void readStatusFromHardware();
  uint8_t isLeft(const std::string &str);

  // ROS Parameters
  std::string serial_port_;
  double polling_timeout_;
  double wheel_diameter_, max_accel_, max_speed_;

  // Store the command for the robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_position_, hw_states_position_offset_, hw_states_velocity_;

  uint8_t left_cmd_joint_index_, right_cmd_joint_index_;

  std::shared_ptr<a200_status::A200Status> status_node_;
  clearpath_platform_msgs::msg::Power power_msg_;
  clearpath_platform_msgs::msg::Status status_msg_;
  clearpath_platform_msgs::msg::StopStatus stop_status_msg_;
  std_msgs::msg::Bool stop_msg_;
  std_msgs::msg::Float32 driver_left_temp_msg_, driver_right_temp_msg_;
  std_msgs::msg::Float32 motor_left_temp_msg_, motor_right_temp_msg_;
};

}  // namespace clearpath_platform

#endif  // CLEARPATH_PLATFORM__A200_HARDWARE_HPP_
