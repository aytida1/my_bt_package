#include "my_bt_package/whine_action.hpp"

#include "rclcpp/rclcpp.hpp"

namespace MyBTNodes
{

WhineAction::WhineAction(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
{
    RCLCPP_INFO(rclcpp::get_logger("WhineAction"), "Constructor: Ready to whine if needed.");
}

BT::PortsList WhineAction::providedPorts()
{
  return {};
}

BT::NodeStatus WhineAction::tick()
{
    RCLCPP_INFO(rclcpp::get_logger("WhineAction"), "'%s' ACTION: Awooo... Sad whine...", this->name().c_str());
    return BT::NodeStatus::SUCCESS; 
}


}