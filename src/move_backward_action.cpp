#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace MyBTNodes
{


class MoveBackwardAction : public BT::SyncActionNode
{
public:
    MoveBackwardAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("move_backward_action");
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        double speed = 0.2;
        double duration = 10.0;
        
        getInput("speed", speed);
        getInput("duration", duration);

        auto start_time = node_->now();
        auto end_time = start_time + rclcpp::Duration::from_nanoseconds(duration * 1e9);

        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.linear.x = -speed;  // Negative for backward movement
        cmd_msg.linear.y = 0.0;
        cmd_msg.linear.z = 0.0;
        cmd_msg.angular.x = 0.0;
        cmd_msg.angular.y = 0.0;
        cmd_msg.angular.z = 0.0;

        while (node_->now() < end_time)
        {
            cmd_vel_pub_->publish(cmd_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }

        // Stop the robot
        cmd_msg.linear.x = 0.0;
        cmd_vel_pub_->publish(cmd_msg);

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

// Register the node
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<MoveBackwardAction>("MoveBackward");
}


}