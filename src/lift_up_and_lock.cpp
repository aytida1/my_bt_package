#include "my_bt_package/lift_up_and_lock.hpp"
#include <limits>

namespace MyBTNodes
{

LiftUpAndLockAction::LiftUpAndLockAction(const std::string& name, const BT::NodeConfiguration& config) 
    : BT::ThreadedAction(name, config), rclcpp::Node("lift_up_and_lock_action"),
      lift_base_joint_position(0.0),  // Initialize variables
      lift_servo_position(0.0)
{
    RCLCPP_INFO(this->get_logger(), "Constructor: Ready to move lift up and lock");

    lift_cmd_publisher_ = this->create_publisher<std_msgs::msg::Float64>("lift_cmd", 10);
    lift_servo_cmd_publisher_ = this->create_publisher<std_msgs::msg::Float64>("lift_servo_cmd", 10);
    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states",
        10,
        std::bind(&LiftUpAndLockAction::action_callback, this, std::placeholders::_1)
    );
}

void LiftUpAndLockAction::action_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
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

BT::PortsList LiftUpAndLockAction::providedPorts()
{
    return {};  // No input/output ports needed
}

BT::NodeStatus LiftUpAndLockAction::tick()
{
    RCLCPP_INFO(this->get_logger(), "Starting lift up and lock procedure...");
    
    // Initialize variables outside the loop (persistent across iterations)
    double last_servo_cmd_ = -999.0;
    double last_lift_cmd_ = -999.0;
    bool servo_locked = false;  // Track servo state
    
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
            RCLCPP_INFO(this->get_logger(), "Current state - Servo: %.6f, Lift: %.6f, Servo locked: %s", 
                        lift_servo_position, lift_base_joint_position, servo_locked ? "YES" : "NO");
        }

        // PHASE 1: Move servo to lock position
        if (!servo_locked) {
            if (lift_servo_position < 1.55) {
                desired_servo_cmd = 0.5;  // Move servo forward
                if (should_log) {
                    RCLCPP_INFO(this->get_logger(), "PHASE 1: Moving servo to lock position... (%.6f/1.55)", lift_servo_position);
                }
            } else {
                // Servo reached target position
                desired_servo_cmd = 0.0;  // Stop servo
                servo_locked = true;
                RCLCPP_INFO(this->get_logger(), "PHASE 1 COMPLETE: Servo locked at position %.6f", lift_servo_position);
            }
        }
        // PHASE 2: Lift up after servo is locked
        else {
            // Keep servo stopped
            desired_servo_cmd = 0.0;
            
            if (lift_base_joint_position < 0.057) {
                desired_lift_cmd = 0.1;  // Move lift up
                if (should_log) {
                    RCLCPP_INFO(this->get_logger(), "PHASE 2: Lifting up... (%.6f/0.57)", lift_base_joint_position);
                }
            } else {
                // Lift reached target position - MISSION COMPLETE
                desired_lift_cmd = 0.0;  // Stop lift
                RCLCPP_INFO(this->get_logger(), "PHASE 2 COMPLETE: Lift reached target position %.6f", lift_base_joint_position);
                
                // Send final stop commands
                std_msgs::msg::Float64 stop_msg;
                stop_msg.data = 0.0;
                lift_cmd_publisher_->publish(stop_msg);
                lift_servo_cmd_publisher_->publish(stop_msg);
                
                RCLCPP_INFO(this->get_logger(), "MISSION COMPLETE: Lift up and lock procedure finished successfully!");
                return BT::NodeStatus::SUCCESS;  // Exit the thread
            }
        }

        // Safety condition: if lift is already very low, don't command it down
        if (lift_base_joint_position <= 0.01) {
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

    RCLCPP_WARN(this->get_logger(), "Lift up and lock action exited unexpectedly");
    return BT::NodeStatus::FAILURE;  // If we exit loop without success
}


} // namespace MyBTNodes