#ifndef BARK_ACTION_HPP_
#define BARK_ACTION_HPP_

#include "behaviortree_cpp/basic_types.h" // For BT::NodeStatus
#include "behaviortree_cpp/tree_node.h"   // Base for all nodes
#include "behaviortree_cpp/bt_factory.h"  // For BT::SyncActionNode (if you use that base)
                                          // Or include "behaviortree_cpp/action_node.h" for general action nodes

// For simple actions that complete immediately, we can use SyncActionNode
#include "behaviortree_cpp/action_node.h" // Defines SyncActionNode

namespace MyBTNodes
{

class BarkAction : public BT::SyncActionNode // Our class 'BarkAction' is a 'SyncActionNode'
{
public:
  BarkAction(const std::string& name, const BT::NodeConfiguration& config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();
};

} // end namespace MyBTNodes

#endif // BARK_ACTION_HPP_