#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

// Include all your custom nodes
#include "my_bt_package/send_nav2_goal_action.hpp"
#include "my_bt_package/lift_up_and_lock.hpp"
#include "my_bt_package/goal_reached_condition.hpp"
#include "my_bt_package/docking_action.hpp"
#include "my_bt_package/unlock_and_lift_down.hpp"
#include "my_bt_package/move_backward_action.hpp"

#include "behaviortree_cpp/loggers/groot2_publisher.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto ros_node = rclcpp::Node::make_shared("bt_tree_runner_node");
    RCLCPP_INFO(ros_node->get_logger(), "Starting Behavior Tree runner application...");

    // Create factory and register nodes
    BT::BehaviorTreeFactory factory;
    
    factory.registerNodeType<MyBTNodes::SendNav2goalAction>("SendNav2GoalAction");
    factory.registerNodeType<MyBTNodes::LiftUpAndLockAction>("LiftUpAndLockAction");
    factory.registerNodeType<MyBTNodes::GoalReachedCondition>("GoalReachedCondition");
    factory.registerNodeType<MyBTNodes::DockingAction>("DockingAction");
    factory.registerNodeType<MyBTNodes::UnlockAndLiftDown>("UnlockAndLiftDownAction");
    factory.registerNodeType<MyBTNodes::MoveBackwardAction>("MoveBackwardAction");

    // Get XML file path
    std::string package_share_directory;
    try {
        package_share_directory = ament_index_cpp::get_package_share_directory("my_bt_package");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(ros_node->get_logger(), "Could not find package 'my_bt_package': %s", e.what());
        return 1;
    }

    std::string tree_xml_path = package_share_directory + "/bt_trees/PsToAsrsToPs.xml";
    RCLCPP_INFO(ros_node->get_logger(), "Loading Behavior Tree from: %s", tree_xml_path.c_str());

    // Create the tree
     BT::Tree tree = factory.createTreeFromFile(tree_xml_path);

    // Setup monitoring and logging
    BT::Groot2Publisher groot2_publisher(tree);
    BT::StdCoutLogger logger_cout(tree);
    
    RCLCPP_INFO(ros_node->get_logger(), "ZMQ Publisher for Groot2 monitoring started.");
    RCLCPP_INFO(ros_node->get_logger(), "Behavior Tree created. Starting execution...");

    // === MAIN EXECUTION LOOP (This was missing!) ===
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    rclcpp::WallRate loop_rate(10.0); // 10 Hz - tick every 100ms
    
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        // Tick the tree
        status = tree.tickOnce();
        
        // Handle different status outcomes
        switch(status) {
            case BT::NodeStatus::SUCCESS:
                RCLCPP_INFO(ros_node->get_logger(), "Behavior Tree completed successfully!");
                break;
            case BT::NodeStatus::FAILURE:
                RCLCPP_ERROR(ros_node->get_logger(), "Behavior Tree failed!");
                break;
            case BT::NodeStatus::RUNNING:
                // Tree is still running, continue
                break;
        }
        
        // Allow ROS to process callbacks (important for your threaded actions)
        rclcpp::spin_some(ros_node);
        
        // Sleep to maintain loop rate
        loop_rate.sleep();
    }

    RCLCPP_INFO(ros_node->get_logger(), "Behavior Tree execution finished!");
    rclcpp::shutdown();
    return 0;
}