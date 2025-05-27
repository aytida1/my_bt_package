#include "my_bt_package/unlock_and_lift_down.hpp"
#include <limits>

namespace MyBTNodes
{

UnlockAndLiftDown::UnlockAndLiftDown(const std::string& name, const BT::NodeConfiguration& config) 
    : BT::ThreadedAction(name, config), rclcpp::Node("unlock_and_lift_down_action"),
      lift_base_joint_position(0.0),  // Initialize variables
      lift_servo_position(0.0)
{
    RCLCPP_INFO(this->get_logger(), "Constructor: Ready to unlock and lift down");

    lift_cmd_publisher_ = this->create_publisher<std_msgs::msg::Float64>("lift_cmd", 10);
    lift_servo_cmd_publisher_ = this->create_publisher<std_msgs::msg::Float64>("lift_servo_cmd", 10);
    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states",
        10,
        std::bind(&UnlockAndLiftDown::action_callback, this, std::placeholders::_1)
    );
}

void UnlockAndLiftDown::action_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Safe array access with bounds checking
    if (msg->position.size() >= 4) {
        lift_base_joint_position = msg->position[2];   // lift_base_joint
        lift_servo_position = msg->position[3];        // lift_servo_motor_left_joint
        
        // Debug logging (comment out after testing)
        RCLCPP_DEBUG(this->get_logger(), "Joint callback - Lift: %.6f, Servo: %.6f", 
                    lift_base_joint_position, lift_servo_position);
    } else {
        RCLCPP_WARN(this->get_logger(), "Joint state message has insufficient position data");
    }
}

BT::PortsList UnlockAndLiftDown::providedPorts()
{
    return {};  // No input/output ports needed
}

BT::NodeStatus UnlockAndLiftDown::tick()
{
    RCLCPP_INFO(this->get_logger(), "Starting unlock and lift down procedure...");
    
    // Initialize variables outside the loop (persistent across iterations)
    double last_servo_cmd_ = -999.0;
    double last_lift_cmd_ = -999.0;
    bool lift_lowered = false;  // Track lift state - first lower lift, then unlock servo
    
    // Main control loop - runs continuously in separate thread
    while (rclcpp::ok()) {
        // Process ROS callbacks to update joint states
        rclcpp::spin_some(this->get_node_base_interface());
        
        std_msgs::msg::Float64 lift_cmd_msg;
        std_msgs::msg::Float64 lift_servo_cmd_msg;
        double desired_servo_cmd = 0.0;
        double desired_lift_cmd = 0.0;

        // Reduce logging spam - only log every 10 iterations (0.5 seconds)
        static int log_counter = 0;
        bool should_log = (log_counter % 10 == 0);
        log_counter++;

        if (should_log) {
            RCLCPP_INFO(this->get_logger(), "Current state - Servo: %.6f, Lift: %.6f, Lift lowered: %s", 
                        lift_servo_position, lift_base_joint_position, lift_lowered ? "YES" : "NO");
        }

        // PHASE 1: Lower lift first
        if (!lift_lowered) {
            if (lift_base_joint_position > 0.01) {
                desired_lift_cmd = -0.1;  // Move lift down
                desired_servo_cmd = 0.0;  // Keep servo locked during lowering
                if (should_log) {
                    RCLCPP_INFO(this->get_logger(), "PHASE 1: Lowering lift... (%.6f/0.01)", lift_base_joint_position);
                }
            } else {
                // Lift reached bottom position
                desired_lift_cmd = 0.0;  // Stop lift
                lift_lowered = true;
                RCLCPP_INFO(this->get_logger(), "PHASE 1 COMPLETE: Lift lowered to position %.6f", lift_base_joint_position);
            }
        }
        // PHASE 2: Unlock servo after lift is down
        else {
            // Keep lift stopped
            desired_lift_cmd = 0.0;
            
            if (lift_servo_position > 0.01) {
                desired_servo_cmd = -0.5;  // Move servo backward to unlock
                if (should_log) {
                    RCLCPP_INFO(this->get_logger(), "PHASE 2: Unlocking servo... (%.6f/0.01)", lift_servo_position);
                }
            } else {
                // Servo reached unlocked position - MISSION COMPLETE
                desired_servo_cmd = 0.0;  // Stop servo
                RCLCPP_INFO(this->get_logger(), "PHASE 2 COMPLETE: Servo unlocked at position %.6f", lift_servo_position);
                
                // Send final stop commands
                std_msgs::msg::Float64 stop_msg;
                stop_msg.data = 0.0;
                lift_cmd_publisher_->publish(stop_msg);
                lift_servo_cmd_publisher_->publish(stop_msg);
                
                RCLCPP_INFO(this->get_logger(), "MISSION COMPLETE: Unlock and lift down procedure finished successfully!");
                return BT::NodeStatus::SUCCESS;  // Exit the thread
            }
        }

        // Safety condition: if lift is already very low, don't command it down further
        if (lift_base_joint_position <= 0.005) {
            desired_lift_cmd = std::max(0.0, desired_lift_cmd);  // Only allow upward movement
        }

        // Only publish if the command has changed
        if (desired_servo_cmd != last_servo_cmd_) {
            lift_servo_cmd_msg.data = desired_servo_cmd;
            lift_servo_cmd_publisher_->publish(lift_servo_cmd_msg);
            last_servo_cmd_ = desired_servo_cmd;
            RCLCPP_DEBUG(this->get_logger(), "Published servo command: %.3f", desired_servo_cmd);
        }

        if (desired_lift_cmd != last_lift_cmd_) {
            lift_cmd_msg.data = desired_lift_cmd;
            lift_cmd_publisher_->publish(lift_cmd_msg);
            last_lift_cmd_ = desired_lift_cmd;
            RCLCPP_DEBUG(this->get_logger(), "Published lift command: %.3f", desired_lift_cmd);
        }

        // Sleep to avoid consuming too much CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    RCLCPP_WARN(this->get_logger(), "Unlock and lift down action exited unexpectedly");
    return BT::NodeStatus::FAILURE;  // If we exit loop without success
}



} // namespace MyBTNodes