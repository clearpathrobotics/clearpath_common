#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <algorithm>
#include <vector>
#include <string>
#include <rclcpp/qos.hpp>
#include <cmath> // For std::abs
#include <mutex>
#include <iostream>
#include <iterator>
#include <array>

class GazeboToRealRobotBridge : public rclcpp::Node
{
public:
    GazeboToRealRobotBridge()
        : Node("gazebo_to_real_robot_bridge")
    {
        real_arm_joint_positions_.resize(7, 0.0);
        // Subscribe to the Gazebo joint states
        gazebo_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/a200_0000/platform/joint_states", 10,
            std::bind(&GazeboToRealRobotBridge::gazebo_joint_states_callback, this, std::placeholders::_1));
        // Set the QoS profile for the publisher to match the expected settings
        rclcpp::QoS qos_profile = rclcpp::QoS(10)
                                    .reliable()               // Reliable QoS
                                    .durability_volatile();   // Volatile durability

        // Subscribe to the /joint_states topic
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&GazeboToRealRobotBridge::joint_state_callback, this, std::placeholders::_1));

        // Publisher for the physical robot's joint trajectory controller
        robot_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", qos_profile);

        // Define the arm joint names you're interested in
        sim_arm_joint_names_ = {
            "arm_0_joint_1",
            "arm_0_joint_2",
            "arm_0_joint_3",
            "arm_0_joint_4",
            "arm_0_joint_5",
            "arm_0_joint_6",
            "arm_0_joint_7"};
        real_arm_joint_names_ = {
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
            "joint_7"  
        };
    }

private:


    // Callback function to handle the incoming joint states from Gazebo
    void gazebo_joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_positions_mutex); // Lock the mutex to prevent race conditions
        if(!init_reached){
            rclcpp::sleep_for(std::chrono::milliseconds(1000));
        }
        
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.joint_names = real_arm_joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint traj_point;

        // Filter only the arm joint positions from the incoming JointState message
        for (const auto &sim_arm_joint_name : sim_arm_joint_names_)
        {
            auto it = std::find(msg->name.begin(), msg->name.end(), sim_arm_joint_name);
            if (it != msg->name.end())
            {
                // Get the index of the arm joint in the message
                auto index = std::distance(msg->name.begin(), it);

                // Append the corresponding position to the trajectory point
                traj_point.positions.push_back(msg->position[index]);
            }
        }

        if(!init_reached){

            std::cout << "Joints positional commands" << std::endl;
            std::copy(traj_point.positions.begin(), traj_point.positions.end(), std::ostream_iterator<float>(std::cout, " "));
            std::cout << "" << std::endl;
            std::cout << "Real-life joints positons" << std::endl;
            std::for_each(std::begin(real_arm_joint_positions_), std::end(real_arm_joint_positions_), [](double val) { std::cout << val << " "; });
            std::cout << "" << std::endl;
            // Calculate the time from start based on the difference between real and simulated joint positions
            double max_position_difference = 0.0;

            for (size_t i = 0; i < traj_point.positions.size(); ++i)
            {
                // Calculate the absolute difference between real and simulated joint positions
                double position_difference = std::abs(traj_point.positions[i] - real_arm_joint_positions_[i]);
                if (position_difference > max_position_difference) {
                    max_position_difference = position_difference;  // Track the maximum difference
                }
            }

            std::cout << "Max position difference: " << max_position_difference << std::endl;

            if (max_position_difference <= tol){
                std::cout << "Robot sync with the simulation initial position reached!" << std::endl;
                init_reached = true;
            }

            // Define a scaling factor for time (adjust this value as necessary)
            double scaling_factor = 3.0; // This controls how much time is given based on the max difference

            // Calculate time based on the difference
            double total_time = max_position_difference * scaling_factor;
            // Set time_from_start based on the maximum position difference
            traj_point.time_from_start.sec = static_cast<int>(total_time);
            traj_point.time_from_start.nanosec = static_cast<int>((total_time - traj_point.time_from_start.sec) * 1e9);


        }
        else{
            traj_point.time_from_start.sec = 0;
        }

        // Add the trajectory point to the message
        traj_msg.points.push_back(traj_point);

        // Publish the trajectory message to the physical robot
        robot_pub_->publish(traj_msg);
    }

    // Callback function to handle joint state messages coming from the rl robot
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_positions_mutex); // Lock the mutex to prevent race conditions
        
        // Ensure that real_arm_joint_positions_ has the correct size (already resized)
        real_arm_joint_positions_.clear();
        real_arm_joint_positions_.resize(real_arm_joint_names_.size(), 0.0);
        

        // Loop through the real_arm_joint_names_ and update the corresponding positions directly
        for (size_t i = 0; i < real_arm_joint_names_.size(); ++i)
        {
            auto it = std::find(msg->name.begin(), msg->name.end(), real_arm_joint_names_[i]);
            if (it != msg->name.end())
            {
                // Get the index of the arm joint in the message
                auto index = std::distance(msg->name.begin(), it);

                // Assign the corresponding position directly to the vector
                real_arm_joint_positions_[i] = msg->position[index];
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gazebo_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr robot_pub_;

    // List of arm joint names you're interested in
    std::vector<std::string> real_arm_joint_names_;
    std::vector<std::string> sim_arm_joint_names_;
    // Vector to store the real-life robot's joints positions
    std::vector<double> real_arm_joint_positions_;
    // Mutex to protect access to real_arm_joint_positions_
    std::mutex joint_positions_mutex;
    bool init_reached = false;
    double tol = 0.04;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GazeboToRealRobotBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
