#include "my_bt_package/send_nav2_goal_action.hpp"
#include "rclcpp/rclcpp.hpp"


namespace MyBTNodes
{

SendNav2goalAction::SendNav2goalAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) , rclcpp::Node("send_nav2_goal_action")
{
    RCLCPP_INFO(rclcpp::get_logger("SendNav2goalAction"), "Constructor: Ready to send Nav2 goal!");

    // nav2 goal publihser
    nav2_goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
}

BT::PortsList SendNav2goalAction::providedPorts(){
    return {};
}

BT::NodeStatus SendNav2goalAction::tick()
{
    auto goal_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    
    // staging pose - see in map and update it !!!
    goal_msg->header.frame_id = "map";
    goal_msg->header.stamp = this->now();
    goal_msg->pose.position.x = 2.0;
    goal_msg->pose.position.y = 0.0;
    goal_msg->pose.position.z = 0.0;
    goal_msg->pose.orientation.x = 0.0;
    goal_msg->pose.orientation.y = 0.0;
    goal_msg->pose.orientation.z = 0.0;
    goal_msg->pose.orientation.w = 0.0;

    nav2_goal_publisher_->publish(*goal_msg);

    return BT::NodeStatus::SUCCESS;

}
}