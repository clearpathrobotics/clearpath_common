#ifndef CLEARPATH_PLATFORM__PUMA_DRIVE_HARDWARE_INTERFACE_HPP_
#define CLEARPATH_PLATFORM__PUMA_DRIVE_HARDWARE_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "puma_motor_msgs/msg/feedback.hpp"
#include "puma_motor_msgs/msg/multi_feedback.hpp"

namespace clearpath_platform
{

class PumaHardwareInterface
: public rclcpp::Node
{
  public:
  explicit PumaHardwareInterface(std::string node_name);

  void drive_command(const sensor_msgs::msg::JointState msg);

  bool has_new_feedback();
  void feedback_callback(const puma_motor_msgs::msg::MultiFeedback::SharedPtr msg);
  puma_motor_msgs::msg::MultiFeedback get_feedback();

  private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_cmd_;
  rclcpp::Subscription<puma_motor_msgs::msg::MultiFeedback>::SharedPtr sub_feedback_;

  puma_motor_msgs::msg::MultiFeedback feedback_;
  std::atomic_bool has_feedback_;
};

} // namespace clearpath_platform

#endif // CLEARPATH_PLATFORM__PUMA_DRIVE_HARDWARE_INTERFACE_HPP_
