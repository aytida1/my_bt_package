#include "my_bt_package/docking_action.hpp"

namespace MyBTNodes
{
DockingAction::DockingAction(const std::string& name, const BT::NodeConfiguration& config) 
 : BT::ThreadedAction(name, config), rclcpp::Node("docking_action")
{
    detection_subscriber_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "detections",
        10, 
        std::bind(&DockingAction::callback_function, this, std::placeholders::_1)
    );

    vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10
    );



}

void DockingAction::callback_function(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
    for (const auto& detection : msg->detections)
    {
        pos_x = detection.
    }
}




}

