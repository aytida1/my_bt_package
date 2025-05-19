#include "my_bt_package/bark_action.hpp"
#include "rclcpp/rclcpp.hpp"

namespace MyBTNodes
{

BarkAction::BarkAction(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
  RCLCPP_INFO(rclcpp::get_logger("BarkAction"), "Constructor: Ready to bark!");
}

BT::PortsList BarkAction::providedPorts()
{
  return {}; // No ports needed for this simple action
}

BT::NodeStatus BarkAction::tick()
{
  RCLCPP_INFO(rclcpp::get_logger("BarkAction"), "'%s' ACTION: Woof woof! Happy bark!", this->name().c_str());
  return BT::NodeStatus::SUCCESS; // Barking is always a success for this example
}

} // end namespace MyBTNodes