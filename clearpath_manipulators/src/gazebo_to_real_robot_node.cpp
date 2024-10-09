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
        : Node("gazebo_to_real_robot_bridge"),
          init_reached(false),
          tol(0.02),
          max_position_difference(0.0),
          position_difference(0.0),
          scaling_factor(3.0), // Initialize with default
          total_time(0.0),
          num_joints(7), // Initialize num_joints here
          real_arm_joint_positions_(num_joints, 0.0), // Initialize vectors in initializer list
          new_positions(num_joints, 0.0),
          real_joint_positions_(num_joints, 0.0)
    {
        // Set the QoS profile for the publisher to match the expected settings
        rclcpp::QoS qos_profile = rclcpp::QoS(10)
                                    .reliable()               // Best effort QoS
                                    .durability_volatile();   // Volatile durability
        // Subscribe to the Gazebo joint states
        gazebo_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/a200_0000/platform/joint_states",
            qos_profile,
            std::bind(&GazeboToRealRobotBridge::gazebo_joint_states_callback, this, std::placeholders::_1));


        // Subscribe to the /joint_states topic
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            qos_profile,
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
        if(!init_reached){
            rclcpp::sleep_for(std::chrono::milliseconds(1000));
        }
        
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.joint_names = real_arm_joint_names_;
        // Resize the points vector to hold exactly one point
        traj_msg.points.resize(1);
        // Reference to the single point for easy access
        trajectory_msgs::msg::JointTrajectoryPoint &traj_point = traj_msg.points[0];
        traj_point.positions.resize(real_arm_joint_names_.size(), 0.0); // Pre-allocate space

        // Extract arm joint positions from the incoming message without holding a lock
        for (size_t i = 0; i < sim_arm_joint_names_.size(); ++i)
        {
            auto it = std::find(msg->name.begin(), msg->name.end(), sim_arm_joint_names_[i]);
            if (it != msg->name.end())
            {
                // Get the index of the arm joint in the message
                auto index = std::distance(msg->name.begin(), it);

                // Directly assign the corresponding position to the pre-sized vector
                traj_point.positions[i] = msg->position[index];
            }
        }
        // Copy real joint positions while holding the lock
        {
            std::lock_guard<std::mutex> lock(joint_positions_mutex);
            real_joint_positions_ = real_arm_joint_positions_;
        }        

        if(!init_reached){

            //std::cout << "Joints positional commands" << std::endl;
            //std::copy(traj_point.positions.begin(), traj_point.positions.end(), std::ostream_iterator<float>(std::cout, " "));
            //std::cout << "" << std::endl;
            //std::cout << "Real-life joints positons" << std::endl;
            //std::for_each(std::begin(real_arm_joint_positions_), std::end(real_arm_joint_positions_), [](double val) { std::cout << val << " "; });
            //std::cout << "" << std::endl;
            // Calculate the time from start based on the difference between real and simulated joint positions
            max_position_difference = 0.0;
            for (size_t i = 0; i < traj_point.positions.size(); ++i)
            {
                // Calculate the absolute difference between real and simulated joint positions
                position_difference = std::abs(traj_point.positions[i] - real_joint_positions_[i]);
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
            scaling_factor = 3.0; // This controls how much time is given based on the max difference

            // Calculate time based on the difference
            total_time = max_position_difference * scaling_factor;
            // Set time_from_start based on the maximum position difference
            traj_point.time_from_start.sec = static_cast<int>(total_time);
            traj_point.time_from_start.nanosec = static_cast<int>((total_time - traj_point.time_from_start.sec) * 1e9);


        }
        else{
            traj_point.time_from_start.sec = 0;
            traj_point.time_from_start.nanosec = 0;
        }

        // Publish the trajectory message to the physical robot
        robot_pub_->publish(traj_msg);
    }

    // Callback function to handle joint state messages coming from the rl robot
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {

        // Extract joint positions without holding the lock
        for (size_t i = 0; i < real_arm_joint_names_.size(); ++i)
        {
            auto it = std::find(msg->name.begin(), msg->name.end(), real_arm_joint_names_[i]);
            if (it != msg->name.end())
            {
                auto index = std::distance(msg->name.begin(), it);
                new_positions[i] = msg->position[index];
            }
        }

        // Now lock the mutex and update the shared vector
        {
            std::lock_guard<std::mutex> lock(joint_positions_mutex);
            real_arm_joint_positions_ = new_positions;
        }
    }


    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gazebo_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr robot_pub_;

    size_t num_joints = 7;
    // List of arm joint names you're interested in
    std::vector<std::string> real_arm_joint_names_;
    std::vector<std::string> sim_arm_joint_names_;
    // Vector to store the real-life robot's joints positions
    std::vector<double> real_arm_joint_positions_;
    // Mutex to protect access to real_arm_joint_positions_
    std::mutex joint_positions_mutex;
    // Temporary variables to reduce the lock scope
    std::vector<double> new_positions;
    std::vector<double> real_joint_positions_;

    bool init_reached;
    double tol;
    double max_position_difference;
    double position_difference;
    double scaling_factor;
    double total_time;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GazeboToRealRobotBridge>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();  // Use multi-threaded spinning
    rclcpp::shutdown();
    return 0;
}