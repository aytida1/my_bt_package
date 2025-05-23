#ifndef DOCKING_ACTION_HPP_
#define DOCKING_ACTION_HPP_

#include "rclcpp/rclcpp.hpp"
// #include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "chrono"

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
    // rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

    void assign_pose();

    // TF2 components for pose between apriltag and camera link
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr timer_;

    // to store position and orientation of pose between apriltag and camera link
    double how_far, how_shift;
    double ori_x, ori_y, ori_z, ori_w;


};

}

#endif 