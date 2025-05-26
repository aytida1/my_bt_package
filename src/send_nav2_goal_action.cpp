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

BT::PortsList SendNav2goalAction::providedPorts()
{
    return {
        BT::InputPort<double>("goal_x", "X coordinate of the goal position"),
        BT::InputPort<double>("goal_y", "Y coordinate of the goal position"),
        BT::InputPort<double>("goal_yaw", 0.0, "Yaw orientation of the goal (optional, default: 0.0)")
    };
}

BT::NodeStatus SendNav2goalAction::tick()
{
    // Get the coordinates from the BehaviorTree ports
    auto goal_x_result = getInput<double>("goal_x");
    auto goal_y_result = getInput<double>("goal_y");
    auto goal_yaw_result = getInput<double>("goal_yaw");

    // Check if required inputs are provided
    if (!goal_x_result || !goal_y_result)
    {
        RCLCPP_ERROR(this->get_logger(), "Missing required input: goal_x and/or goal_y");
        return BT::NodeStatus::FAILURE;
    }

    double goal_x = goal_x_result.value();
    double goal_y = goal_y_result.value();
    double goal_yaw = goal_yaw_result.value_or(0.0);  // Default to 0.0 if not provided

    // Create the goal message
    auto goal_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    
    goal_msg->header.frame_id = "map";
    goal_msg->header.stamp = this->now();
    goal_msg->pose.position.x = goal_x;
    goal_msg->pose.position.y = goal_y;
    goal_msg->pose.position.z = 0.0;
    
    // Convert yaw to quaternion
    goal_msg->pose.orientation.x = 0.0;
    goal_msg->pose.orientation.y = 0.0;
    goal_msg->pose.orientation.z = sin(goal_yaw / 2.0);
    goal_msg->pose.orientation.w = cos(goal_yaw / 2.0);

    nav2_goal_publisher_->publish(*goal_msg);

    RCLCPP_INFO(this->get_logger(), "Published goal: x=%.2f, y=%.2f, yaw=%.2f", 
                goal_x, goal_y, goal_yaw);

    return BT::NodeStatus::SUCCESS;
}
}