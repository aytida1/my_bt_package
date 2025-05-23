#include "my_bt_package/docking_action.hpp"
#include <tf2/utils.h>

using namespace std::chrono_literals;

namespace MyBTNodes
{
DockingAction::DockingAction(const std::string& name, const BT::NodeConfiguration& config) 
 : BT::ThreadedAction(name, config), rclcpp::Node("docking_action")
{
    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10
    );

    // Create timer to call getAprilTagPose every 10 seconds
    timer_ = this->create_wall_timer(
        100ms, std::bind(&DockingAction::assign_pose, this));



}


void DockingAction::assign_pose()
{
    geometry_msgs::msg::TransformStamped transform = 
            tf_buffer_->lookupTransform("base_link", "tag36h11:0", tf2::TimePointZero);

    how_far = transform.transform.translation.x;   // I think this will be how far is tag
    how_shift = transform.transform.translation.y;   // this will be how shifted to tag is

    // Extract quaternion components
    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);

    // Convert quaternion to Euler angles (roll, pitch, yaw)
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
}



}

