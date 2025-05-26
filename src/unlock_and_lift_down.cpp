#include "my_bt_package/unlock_and_lift_down.hpp"


namespace MyBTNodes
{

UnlockAndLiftDown::UnlockAndLiftDown(const std::string& name, const BT::NodeConfiguration& config) 
    : BT::ThreadedAction(name, config), rclcpp::Node("lift_up_and_lock_action")
{
    RCLCPP_INFO(rclcpp::get_logger("SendNav2goalAction"), "Constructor: Ready to move lift up and lock");

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
    lift_base_joint_position = msg->position[2];   
    // header:
    //     stamp:
    //         sec: 112
    //         nanosec: 308000000
    //     frame_id: ''
    //     name:
    //     - base_left_wheel_joint
    //     - base_right_wheel_joint
    //     - lift_base_joint          <----- at index 2
    //     - lift_servo_motor_left_joint
    //     - lift_servo_motor_right_joint
    //     position:
    //     - 4.3020118396623395e-18
    //     - 2.1241313321140522e-18
    //     - -1.1414000126970886e-14
    //     - 1.069443960180965e-18
    //     - 2.466652912976766e-19
    //     velocity:
    //     - 2.5523648089608082e-18
    //     - -1.1663304388514098e-18
    //     - 4.51028106856638e-19
    //     - 2.169302545893668e-20
    //     - 2.0770984192131016e-20
    //     effort:
    //     - 0.0
    //     - 0.0
    //     - 0.0
    //     - 0.0
    //     - 0.0
    //     ---

}

BT::PortsList UnlockAndLiftDown::providedPorts(){
    return {};
}


BT::NodeStatus UnlockAndLiftDown::tick()
{
    RCLCPP_INFO(this->get_logger(), "Starting unlock and lift down procedure...");
    
    // Initialize variables outside the loop
    double last_servo_cmd_ = std::numeric_limits<double>::quiet_NaN();
    double last_lift_cmd_ = std::numeric_limits<double>::quiet_NaN();
    
    // Main control loop - runs continuously in separate thread
    while (rclcpp::ok()) {
        std_msgs::msg::Float64 lift_cmd_msg;
        std_msgs::msg::Float64 lift_servo_cmd_msg;
        bool move_lift_down = false;
        double desired_servo_cmd = 0.0;
        double desired_lift_cmd = 0.0;
        bool change_status = false;

        // a condition to fix the bug of joint controller
        if (lift_base_joint_position <= 0.01) {
            desired_lift_cmd = 0.0;
        }

        // move servo to lock position
        if (lift_servo_position > 0.01) {
            desired_servo_cmd = -0.5;
        } else {
            desired_servo_cmd = 0.0;
            move_lift_down = true;
        }

        if (move_lift_down && (lift_base_joint_position > 0.01)) {
            desired_lift_cmd = -0.1;
        } else {
            desired_lift_cmd = 0.0;
            move_lift_down = false;
            change_status = true;
        }

        // Only publish if the command has changed
        if (std::isnan(last_servo_cmd_) || desired_servo_cmd != last_servo_cmd_) {
            lift_servo_cmd_msg.data = desired_servo_cmd;
            lift_servo_cmd_publisher_->publish(lift_servo_cmd_msg);
            last_servo_cmd_ = desired_servo_cmd;
        }

        if (std::isnan(last_lift_cmd_) || desired_lift_cmd != last_lift_cmd_) {
            lift_cmd_msg.data = desired_lift_cmd;
            lift_cmd_publisher_->publish(lift_cmd_msg);
            last_lift_cmd_ = desired_lift_cmd;
        }

        // Check if action is complete
        if (change_status) {
            RCLCPP_INFO(this->get_logger(), "Unlock and lift down completed successfully!");
            return BT::NodeStatus::SUCCESS;  // Exit the thread
        } 

        // Sleep to avoid consuming too much CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return BT::NodeStatus::FAILURE;  // If we exit loop without success
}


}