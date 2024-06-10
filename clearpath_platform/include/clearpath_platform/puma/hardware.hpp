#ifndef CLEARPATH_PLATFORM_PUMA__HARDWARE_HPP_
#define CLEARPATH_PLATFORM_PUMA__HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/visibility_control.h"

#include "clearpath_platform/puma/hardware_interface.hpp"


namespace clearpath_platform
{

static constexpr uint8_t DIFF_DRIVE_TWO_JOINTS = 2;
static constexpr uint8_t DIFF_DRIVE_FOUR_JOINTS = 4;
static constexpr double MINIMUM_VELOCITY = 0.01f;


class PumaHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PumaHardware)

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
protected:
  void writeCommandsToHardware();
  void updateJointsFromHardware();
  virtual hardware_interface::CallbackReturn getHardwareInfo(const hardware_interface::HardwareInfo & info);
  virtual hardware_interface::CallbackReturn validateJoints();
  virtual hardware_interface::CallbackReturn initHardwareInterface();
  std::shared_ptr<PumaHardwareInterface> node_;

  // Store the command for the robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_position_, hw_states_position_offset_, hw_states_velocity_;

  std::map<std::string, uint8_t> wheel_joints_;

  uint8_t num_joints_;
  std::string hw_name_;
};

}  // namespace clearpath_platform

#endif  // CLEARPATH_PLATFORM_PUMA_HARDWARE_HPP_
