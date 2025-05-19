#ifndef IS_TOY_PRESENT_CONDITION_HPP_ // These lines prevent the file from being included multiple times by mistake
#define IS_TOY_PRESENT_CONDITION_HPP_

#include "behaviortree_cpp/condition_node.h"

namespace MyBTNodes 
{
class IsToyPresentCondition : public BT::ConditionNode
{
public:
    IsToyPresentCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts(){
        return {BT::InputPort<bool>("is_toy_really_present")};
    };

private:
    // bool toy_present_simulation_ = true;


};


}

#endif