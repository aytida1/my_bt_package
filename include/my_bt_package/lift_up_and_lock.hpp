#ifndef LIFT_UP_AND_LOCK_HPP_
#define LIFT_UP_AND_LOCK_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "behaviortree_cpp/basic_types.h" // For BT::NodeStatus

#include "behaviortree_cpp/action_node.h"

namespace MyBTNodes
{
class LiftUpAndLockAction : public BT::ThreadedAction, public rclcpp::Node
{
public:
    // constructor
    LiftUpAndLockAction(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lift_cmd_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lift_servo_cmd_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

    void action_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    float lift_base_joint_position;
    float lift_servo_position;
};


}


#endif