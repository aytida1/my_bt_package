#include "my_bt_package/docking_action.hpp"
#include <tf2/utils.h>

using namespace std::chrono_literals;

namespace MyBTNodes
{
DockingAction::DockingAction(const std::string& name, const BT::NodeConfiguration& config) 
 : BT::ThreadedAction(name, config), rclcpp::Node("docking_action")
{
    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10
    );

    // Create timer to call getAprilTagPose every 10 seconds
    // timer_ = this->create_wall_timer(
    //     100ms, std::bind(&DockingAction::assign_pose, this));



}

BT::PortsList DockingAction::providedPorts(){
    return {};
};


// void DockingAction::assign_pose()
// {
//     // geometry_msgs::msg::TransformStamped transform = 
//     //         tf_buffer_->lookupTransform("base_link", "tag36h11:0", tf2::TimePointZero);

//     // how_far = transform.transform.translation.x;   // I think this will be how far is tag
//     // how_shift = transform.transform.translation.y;   // this will be how shifted to tag is

//     // // Extract quaternion components
//     // tf2::Quaternion q(
//     //     transform.transform.rotation.x,
//     //     transform.transform.rotation.y,
//     //     transform.transform.rotation.z,
//     //     transform.transform.rotation.w);

//     // // Convert quaternion to Euler angles (roll, pitch, yaw)
//     // double roll, pitch, yaw;
//     // tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
// }


BT::NodeStatus DockingAction::tick()
{
    RCLCPP_INFO(this->get_logger(), "Starting docking procedure...");
    
    // This entire method runs in a separate thread
    while (rclcpp::ok()) {
        try {
            // Get current transform (this will use the latest data from assign_pose)
            geometry_msgs::msg::TransformStamped transform = 
                tf_buffer_->lookupTransform("base_link", "tag36h11:0", tf2::TimePointZero);
            
            how_far = transform.transform.translation.x;
            how_shift = transform.transform.translation.y;
            
            // Convert to yaw
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            
            // Check if we're close enough (success condition)
            if (std::abs(how_far) < 0.1 && std::abs(how_shift) < 0.005) {
                // Stop the robot
                auto stop_msg = geometry_msgs::msg::Twist();
                vel_publisher_->publish(stop_msg);
                
                RCLCPP_INFO(this->get_logger(), "Docking completed successfully!");
                return BT::NodeStatus::SUCCESS;  // This ends the threaded action
            } else {
                return BT::NodeStatus::RUNNING;
            }
            
            // Create velocity command for docking
            auto vel_msg = geometry_msgs::msg::Twist();
            // Calculate distance error (target is 0.1m)
            double distance_error = how_far - 0.1;

            // Linear velocity - proportional control with minimum speed
            if (distance_error > 0.05) {
                vel_msg.linear.x = std::max(0.05, std::min(0.2, distance_error * 0.8));
            } else if (distance_error < -0.02) {
                vel_msg.linear.x = std::max(-0.1, distance_error * 0.5);  // Reverse if too close
            } else {
                vel_msg.linear.x = 0.0;  // Stop when close enough
            }

            // Angular velocity - weighted combination of yaw and lateral corrections
            double yaw_weight = 0.6;
            double lateral_weight = 2.5;

            // Target lateral offset is 0 (between +0.005 and -0.005)
            double lateral_error = how_shift;  // Current offset from center

            double yaw_correction = -yaw * yaw_weight;
            double lateral_correction = -lateral_error * lateral_weight;

            // Combine corrections with saturation
            vel_msg.angular.z = std::max(-0.5, std::min(0.5, yaw_correction + lateral_correction));

            vel_publisher_->publish(vel_msg);
            
            // Check for timeout or failure conditions
            // if (some_failure_condition) {
            //     return BT::NodeStatus::FAILURE;
            // }
            
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
            // Continue trying...
        }
        
        // Sleep to avoid consuming too much CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    return BT::NodeStatus::FAILURE;  // If we exit the loop without success


}



}

