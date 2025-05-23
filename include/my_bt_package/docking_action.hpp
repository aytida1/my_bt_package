#ifndef DOCKING_ACTION_HPP_
#define DOCKING_ACTION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "behaviortree_cpp/basic_types.h" // For BT::NodeStatus
#include "behaviortree_cpp/action_node.h"


namespace MyBTNodes
{


class DockingAction : public BT::ThreadedAction, public rclcpp::Node
{
public:
    // constructor
    DockingAction(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();


private:
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

    void callback_function(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);

    // to store position and orientation of pose between apriltag and camera link
    double pos_x, pos_y;
    double ori_x, ori_y, ori_z, ori_w;


};

}

#endif 