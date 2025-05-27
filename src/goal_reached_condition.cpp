#include "my_bt_package/goal_reached_condition.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace MyBTNodes
{

// constructor
GoalReachedCondition::GoalReachedCondition(const std::string& name, const BT::NodeConfiguration& config) 
    : BT::ConditionNode(name, config), 
      navigation_active_(false), 
      navigation_succeeded_(false),
      has_received_goal_(false)
{
    // defining node
    node_ = rclcpp::Node::make_shared("goal_result_bt_node");
    
    // Subscribe to goal_pose topic to know when goals are sent
    goal_pose_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10,
        std::bind(&GoalReachedCondition::goal_pose_callback, this, std::placeholders::_1)
    );
    
    // Subscribe to navigation status to monitor completion
    status_subscriber_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
        "/navigate_to_pose/_action/status", 10,
        std::bind(&GoalReachedCondition::status_callback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(rclcpp::get_logger("GoalReachedCondition"), "Monitoring goal_pose topic and navigation status");
}

void GoalReachedCondition::goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // A new goal was sent
    has_received_goal_ = true;
    navigation_active_ = true;
    navigation_succeeded_ = false;
    
    RCLCPP_INFO(rclcpp::get_logger("GoalReachedCondition"), 
                "New goal received: (%.2f, %.2f)", 
                msg->pose.position.x, msg->pose.position.y);
}

void GoalReachedCondition::status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
    if (!has_received_goal_) {
        return;  // Don't process status if we haven't seen a goal yet
    }
    
    if (msg->status_list.empty()) {
        // No active goals - if we had one before, it must be complete
        if (navigation_active_) {
            navigation_succeeded_ = true;
            navigation_active_ = false;
            RCLCPP_INFO(rclcpp::get_logger("GoalReachedCondition"), "Navigation completed - no active goals");
        }
        return;
    }
    
    // Check the most recent goal status
    auto latest_status = msg->status_list.back().status;
    
    switch (latest_status) {
        case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
            navigation_succeeded_ = true;
            navigation_active_ = false;
            RCLCPP_INFO(rclcpp::get_logger("GoalReachedCondition"), "Navigation SUCCEEDED");
            break;
        case action_msgs::msg::GoalStatus::STATUS_ABORTED:
            navigation_succeeded_ = false;
            navigation_active_ = false;
            RCLCPP_ERROR(rclcpp::get_logger("GoalReachedCondition"), "Navigation ABORTED");
            break;
        case action_msgs::msg::GoalStatus::STATUS_CANCELED:
            navigation_succeeded_ = false;
            navigation_active_ = false;
            RCLCPP_WARN(rclcpp::get_logger("GoalReachedCondition"), "Navigation CANCELED");
            break;
        case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
        case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
            navigation_active_ = true;
            navigation_succeeded_ = false;
            RCLCPP_DEBUG(rclcpp::get_logger("GoalReachedCondition"), "Navigation in progress");
            break;
        default:
            // Keep current state
            break;
    }
}

BT::NodeStatus GoalReachedCondition::tick()
{
    // Process ROS callbacks
    rclcpp::spin_some(node_);
    
    if (!has_received_goal_) {
        RCLCPP_DEBUG(rclcpp::get_logger("GoalReachedCondition"), "Waiting for goal to be sent...");
        return BT::NodeStatus::RUNNING;
    }
    
    if (navigation_active_) {
        RCLCPP_DEBUG(rclcpp::get_logger("GoalReachedCondition"), "Navigation in progress...");
        return BT::NodeStatus::RUNNING;
    }
    
    if (navigation_succeeded_) {
        RCLCPP_INFO(rclcpp::get_logger("GoalReachedCondition"), "Goal reached successfully!");
        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("GoalReachedCondition"), "Navigation failed!");
        return BT::NodeStatus::FAILURE;
    }
}

BT::PortsList GoalReachedCondition::providedPorts()
{
    return {};
}

} // namespace MyBTNodes