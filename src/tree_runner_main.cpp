#include "rclcpp/rclcpp.hpp"                        // For ROS 2 logging and basic node functionality
#include "behaviortree_cpp/bt_factory.h"            // For BT::BehaviorTreeFactory
#include "behaviortree_cpp/loggers/bt_cout_logger.h" // For printing tree status to console
#include "ament_index_cpp/get_package_share_directory.hpp" // To find our XML file

// === IMPORTANT: Include the headers for ALL your custom nodes ===
#include "my_bt_package/is_toy_present_condition.hpp"
#include "my_bt_package/bark_action.hpp"
#include "my_bt_package/whine_action.hpp"

#include "behaviortree_cpp/loggers/groot2_publisher.h" // For Groot2 live monitoring

int main(int argc, char **argv)
{
  // Initialize ROS 2. Even if not using many ROS features, it's needed for logging.
  rclcpp::init(argc, argv);

  // Create a simple ROS 2 Node for logging purposes.
  // Behavior Trees can be run without a ROS Node, but logging is convenient via ROS.
  auto ros_node = rclcpp::Node::make_shared("bt_tree_runner_node");
  RCLCPP_INFO(ros_node->get_logger(), "Starting Behavior Tree runner application...");

  // The BT Factory is used to create tree instances and register custom node types.
  BT::BehaviorTreeFactory factory;

  // === Register Your Custom Nodes ===
  // You MUST register all the custom node types that you use in your XML file.
  // The first argument is the "ID" that you used in your Groot2 XML.
  // The second part tells the factory how to create an instance of your C++ class.
  // We are using our namespace 'MyBTNodes' here.
  factory.registerNodeType<MyBTNodes::IsToyPresentCondition>("IsToyPresentCondition");
  factory.registerNodeType<MyBTNodes::BarkAction>("BarkAction");
  factory.registerNodeType<MyBTNodes::WhineAction>("WhineAction");

  // Find the path to our XML file.
  // ROS 2 packages can find their shared files (like our XML) using this.
  std::string package_share_directory;
  try {
    package_share_directory = ament_index_cpp::get_package_share_directory("my_bt_package");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(ros_node->get_logger(), "Could not find package 'my_bt_package': %s", e.what());
    return 1;
  }

  std::string tree_xml_path = package_share_directory + "/bt_trees/toy_reaction_tree.xml";
  RCLCPP_INFO(ros_node->get_logger(), "Loading Behavior Tree from: %s", tree_xml_path.c_str());

  // === Create a Blackboard ===
  BT::Blackboard::Ptr blackboard = BT::Blackboard::create();

  blackboard->set("bb_toy_present_status", true);

  // Create the Behavior Tree from the XML file.
  // The factory looks at the XML, finds the node IDs, and uses the registered C++ classes to build the tree.
  // "MainTreeToExecute" must match the 'main_tree_to_execute' attribute in your XML's <root> tag.
  BT::Tree tree = factory.createTreeFromFile(tree_xml_path, blackboard);

  // -------------for live monitoring in groot2------------------------
    // BT::StdCoutLogger logger_cout(tree); // Keep this for console logs

    // === Add the ZMQ Publisher for Groot2 ===
    // Default ports are 1666 for publishing status changes and 1667 for the server.
    // You can specify different ports if needed.
    // The 25 is the server publish rate in Hz (optional).
    BT::Groot2Publisher groot2_publisher(tree); // Use default ports
    // Or: BT::PublisherZMQ publisher_zmq(tree, 25, 1666, 1667);
    RCLCPP_INFO(ros_node->get_logger(), "ZMQ Publisher for Groot2 monitoring started.");
    //------------------------------------------------------------------
    

  // Add a logger to print transitions to the console. This helps see what the tree is doing.
  BT::StdCoutLogger logger_cout(tree);

  RCLCPP_INFO(ros_node->get_logger(), "Behavior Tree created. Starting execution...");

  // The main loop to "tick" the tree.
  // A Behavior Tree usually runs in a loop. Each "tick" makes it re-evaluate its state.
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  rclcpp::WallRate loop_rate(0.3); // Tick the tree 1 time per second (1 Hz)

  int tick_count = 0;
  // Keep ticking until the tree returns SUCCESS or FAILURE, or until ROS shuts down.
  // For a tree that's meant to run continuously, the root might always return RUNNING,
  // or you might re-tick it regardless of status (e.g., for a fixed number of times or duration).
  while (rclcpp::ok() && tick_count < 10) // Let's run for a few ticks
  {
    tick_count++;
    RCLCPP_INFO(ros_node->get_logger(), "--- Tick %d ---", tick_count);

    if (tick_count == 3){
        RCLCPP_INFO(ros_node->get_logger(), "!!! Oh no, the toy disappeared! Updating blackboard. !!!");
        blackboard->set("bb_toy_present_status", false);
    }

    if (tick_count == 6){
        RCLCPP_INFO(ros_node->get_logger(), "!!! Oh yay, the toy found again! Updating blackboard. !!!");
        blackboard->set("bb_toy_present_status", true);
    }



    status = tree.tickOnce(); // tickOnce() is non-blocking
                               // For blocking tick: status = tree.tickRoot();
    RCLCPP_INFO(ros_node->get_logger(), "Tree status after tick %d: %s", tick_count, BT::toStr(status).c_str());

    // If your nodes had ROS actions/services, you'd spin the ROS node here too.
    // rclcpp::spin_some(ros_node);

    loop_rate.sleep(); // Wait to achieve the 1 Hz rate

    if (status != BT::NodeStatus::RUNNING && tick_count < 9) { // If tree finished but we want to tick more
        RCLCPP_INFO(ros_node->get_logger(), "Tree HALTED. Will tick again to observe blackboard changes.");
        // No explicit reset needed for this simple tree if we just keep ticking from the root.
    } else if (status != BT::NodeStatus::RUNNING) {
        break; // Exit loop if tree halts and we are at the end of desired ticks
    }
  }

  RCLCPP_INFO(ros_node->get_logger(), "Behavior Tree execution finished after %d ticks with status: %s", tick_count, BT::toStr(status).c_str());
  rclcpp::shutdown();
  return 0;
}