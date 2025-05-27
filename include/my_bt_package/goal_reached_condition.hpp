// In goal_reached_condition.hpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace MyBTNodes
{

class GoalReachedCondition : public BT::ConditionNode
{
public:
    GoalReachedCondition(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_subscriber_;
    
    // State variables
    bool navigation_active_;
    bool navigation_succeeded_;
    bool has_received_goal_;
    
    // Callbacks
    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
};

} // namespace MyBTNodes