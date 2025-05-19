#ifndef WHINE_ACTION_NODE_
#define WHINE_ACTION_NODE_


#include "behaviortree_cpp/action_node.h"

namespace MyBTNodes
{
class WhineAction : public BT::SyncActionNode
{
public:
    WhineAction(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();
    

};


}



#endif