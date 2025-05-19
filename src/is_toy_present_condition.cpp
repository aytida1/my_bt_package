#include "my_bt_package/is_toy_present_condition.hpp"
#include "rclcpp/rclcpp.hpp"

namespace MyBTNodes
{

IsToyPresentCondition::IsToyPresentCondition(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config)
{

    RCLCPP_INFO(rclcpp::get_logger("IsToyPresentCondition"), "Constructor for %s", name.c_str());

}


// BT::PortsList IsToyPresentCondition::providedPorts()
// {
//   return {}; // Returns an empty list of ports
// }

BT::NodeStatus IsToyPresentCondition::tick()
{

    //toy status from BLACKBOARD (bb)
    BT::Expected<bool> toy_status_from_bb = getInput<bool>("is_toy_really_present");

    if(!toy_status_from_bb){
        RCLCPP_INFO(rclcpp::get_logger("IsToyPresentCondition"), "'%s' failed to read 'toy_status_from_bb' from BlackBoard : %s", this->name().c_str(), toy_status_from_bb.error().c_str());
    }
    
    if (toy_status_from_bb.value() == true){
        RCLCPP_INFO(rclcpp::get_logger("IsToyPresentCondition"), "'%s' ticked: Toy Is Present👍! Returning Success.", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("IsToyPresentCondition"), "'%s' ticked: Toy Is Not Present👎! Returning Failure.", this->name().c_str());
        return BT::NodeStatus::FAILURE;
    }
    
}





}