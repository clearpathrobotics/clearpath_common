#include "clearpath_platform/puma/hardware_interface.hpp"

using clearpath_platform::PumaHardwareInterface;

/**
 * @brief Construct a new PumaHardwareInterface object
*/
PumaHardwareInterface::PumaHardwareInterface(std::string node_name)
: Node(node_name)
{
  sub_feedback_ = create_subscription<puma_motor_msgs::msg::MultiFeedback>(
    "platform/puma/feedback",
    rclcpp::SensorDataQoS(),
    std::bind(&PumaHardwareInterface::feedback_callback, this, std::placeholders::_1));

  pub_cmd_ = create_publisher<sensor_msgs::msg::JointState>(
    "platform/puma/cmd",
    rclcpp::SensorDataQoS());
}

/**
 * @brief Callback for feedback
 *
 * @param msg
*/
void PumaHardwareInterface::feedback_callback(const puma_motor_msgs::msg::MultiFeedback::SharedPtr msg)
{
  feedback_ = *msg;
  has_feedback_ = true;
}

/**
 * @brief Publish drive command
 *
 * @param
*/
void PumaHardwareInterface::drive_command(const sensor_msgs::msg::JointState msg)
{
  pub_cmd_->publish(msg);
  return;
}

/**
 * @brief Check if there is new feedback
 *
 * @return true
 * @return false
*/
bool PumaHardwareInterface::has_new_feedback()
{
  return has_feedback_;
}

/**
 * @brief Get feedback
 *
 * @return puma_motor_msgs::msg::MultiFeedback
*/
puma_motor_msgs::msg::MultiFeedback PumaHardwareInterface::get_feedback()
{
  has_feedback_ = false;
  return feedback_;
}
