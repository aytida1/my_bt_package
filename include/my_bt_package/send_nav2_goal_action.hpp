#ifndef SEND_NAV2_GOAL_ACTION_HPP_
#define SEND_NAV2_GOAL_ACTION_HPP_

#include "behaviortree_cpp/basic_types.h" // For BT::NodeStatus
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

// For simple actions that complete immediately, we can use SyncActionNode
#include "behaviortree_cpp/action_node.h" // Defines SyncActionNode

namespace MyBTNodes
{

class SendNav2goalAction : public BT::SyncActionNode, public rclcpp::Node
{
public:
    SendNav2goalAction(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav2_goal_publisher_;
};  

} // end namespace MyBTNodes

#endif 