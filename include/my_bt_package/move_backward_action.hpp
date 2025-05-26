#ifndef MY_BT_PACKAGE_MOVE_BACKWARD_ACTION_HPP
#define MY_BT_PACKAGE_MOVE_BACKWARD_ACTION_HPP

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace MyBTNodes
{

class MoveBackwardAction : public BT::SyncActionNode
{
public:
    MoveBackwardAction(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    double duration_;
    double speed_;
};

} // namespace my_bt_package

#endif // MY_BT_PACKAGE_MOVE_BACKWARD_ACTION_HPP