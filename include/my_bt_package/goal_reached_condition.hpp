#ifndef GOAL_REACHED_CONDITION_HPP_
#define GOAL_REACHED_CONDITION_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class GoalReachedCondition : public BT::ConditionNode
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    // constuctor
    GoalReachedCondition(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts(){
        return {};
    }

    BT::NodeStatus tick() override;

private:
    bool has_result_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::Node::SharedPtr node_;
    void handle_result_response(const GoalHandle::WrappedResult &result);
    rclcpp_action::ResultCode result_code;
};

#endif