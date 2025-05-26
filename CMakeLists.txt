cmake_minimum_required(VERSION 3.8)
project(my_bt_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Executable
add_executable(tree_runner_exe src/tree_runner_main.cpp)
add_executable(tree_runner_PsToAsrsToPs src/tree_runner_PsToAsrsToPs.cpp)

# Custom BT Node Library
add_library(my_custom_bt_nodes SHARED
  src/bark_action.cpp
  src/is_toy_present_condition.cpp
  src/whine_action.cpp
  src/docking_action.cpp
  src/goal_reached_condition.cpp
  src/lift_up_and_lock.cpp
  src/move_backward_action.cpp
  src/send_nav2_goal_action.cpp
  src/unlock_and_lift_down.cpp
  )

# Include directories for custom nodes
target_include_directories(my_custom_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies to custom node library
ament_target_dependencies(my_custom_bt_nodes PUBLIC 
  behaviortree_cpp
  rclcpp
  ament_index_cpp
  geometry_msgs
  nav2_msgs
  std_msgs
  tf2_ros
  sensor_msgs
  tf2
)

ament_target_dependencies(tree_runner_exe PUBLIC
  rclcpp
  behaviortree_cpp
  ament_index_cpp
)

ament_target_dependencies(tree_runner_PsToAsrsToPs PUBLIC
  rclcpp
  behaviortree_cpp
  ament_index_cpp
  geometry_msgs
  nav2_msgs
  std_msgs
  tf2_ros
  sensor_msgs
  tf2
)

target_link_libraries(tree_runner_exe PUBLIC my_custom_bt_nodes)
target_link_libraries(tree_runner_PsToAsrsToPs PUBLIC my_custom_bt_nodes)


# Install executable
install(TARGETS tree_runner_exe tree_runner_PsToAsrsToPs
  DESTINATION lib/${PROJECT_NAME}
)

# Install custom node library
install(TARGETS my_custom_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install Behavior Tree XML files
install(DIRECTORY
  bt_trees/
  DESTINATION share/${PROJECT_NAME}/bt_trees
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  behaviortree_cpp
  ament_index_cpp
  geometry_msgs
  nav2_msgs
  std_msgs
  tf2_ros
  sensor_msgs
  tf2
)

# Linting and testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
