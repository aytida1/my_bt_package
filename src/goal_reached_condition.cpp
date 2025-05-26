#include "my_bt_package/goal_reached_condition.hpp"


namespace MyBTNodes
{

    
 // namespace MyBTNodes


// constructor
GoalReachedCondition::GoalReachedCondition( const std::string& name, const BT::NodeConfiguration& config) 
: BT::ConditionNode(name, config), has_result_(false)
{
    // defingin node
    node_ = rclcpp::Node::make_shared("goal_result_bt_node");
    
    // defining action client
    client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
    
}



void GoalReachedCondition::handle_result_response(const GoalHandle::WrappedResult &result)
{
    result_code = result.code;
    has_result_ = true;
}

BT::NodeStatus GoalReachedCondition::tick()
{
    // if no result yet then flag running
    if(!has_result_)
    {
        return BT::NodeStatus::RUNNING;
    }

    // when got result
    switch(result_code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            return BT::NodeStatus::SUCCESS;
        case rclcpp_action::ResultCode::ABORTED:
            return BT::NodeStatus::FAILURE;
        case rclcpp_action::ResultCode::CANCELED:
            return BT::NodeStatus::FAILURE;
        default:
            return BT::NodeStatus::RUNNING;
    }

}

}