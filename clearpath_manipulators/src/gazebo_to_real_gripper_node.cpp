#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <iostream>

class GripperSyncNode : public rclcpp::Node
{
public:
    using GripperCommand = control_msgs::action::GripperCommand;

    GripperSyncNode()
        : Node("gazebo_to_real_gripper_bridge")
    {
        // Initialize the action client for the real-life gripper
        real_gripper_client_ = rclcpp_action::create_client<GripperCommand>(this, "/robotiq_gripper_controller/gripper_cmd");

        // Initialize the subscription to the simulated robot's joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/a200_0000/platform/joint_states", 10,
            std::bind(&GripperSyncNode::jointStateCallback, this, std::placeholders::_1));

        // Wait for the action server to become available
        if (!real_gripper_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Real gripper action server not available.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Real gripper action server connected.");
        }

        RCLCPP_INFO(this->get_logger(), "Gripper sync node started.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp_action::Client<GripperCommand>::SharedPtr real_gripper_client_;

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        //rclcpp::sleep_for(std::chrono::milliseconds(1000));
        // Find the index of the 'robotiq_85_left_knuckle_joint'
        auto it = std::find(msg->name.begin(), msg->name.end(), "robotiq_85_left_knuckle_joint");
        if (it != msg->name.end())
        {
            int index = std::distance(msg->name.begin(), it);
            double joint_position = msg->position[index];
            sendGripperCommand(joint_position);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Joint 'robotiq_85_left_knuckle_joint' not found.");
        }
    }

    void sendGripperCommand(double position)
    {
        // Prepare the gripper command goal
        auto goal_msg = GripperCommand::Goal();
        goal_msg.command.position = position;  // Map joint position directly to the gripper command
        goal_msg.command.max_effort = 100.0;

        //rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions send_goal_options;

        // Active callback for goal response
        /*
        send_goal_options.goal_response_callback = 
            [this](const rclcpp_action::Client<control_msgs::action::GripperCommand>::GoalHandle::SharedPtr& goal_handle) {
                if (!goal_handle)
                {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Gripper command sent successfully.");
                }
            };
        */
        // Result callback for goal completion
        /*
        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<GripperCommand>::WrappedResult & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "Gripper command succeeded.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Gripper command failed with code: %d", result.code);
                }
            };
        */

        // Send the goal to the real gripper
        real_gripper_client_->async_send_goal(goal_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperSyncNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
