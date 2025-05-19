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

# Executable
add_executable(tree_runner_exe src/tree_runner_main.cpp)

# Custom BT Node Library
add_library(my_custom_bt_nodes SHARED
  src/bark_action.cpp
  src/is_toy_present_condition.cpp
  src/whine_action.cpp
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
)

ament_target_dependencies(tree_runner_exe PUBLIC
  rclcpp
  behaviortree_cpp
  ament_index_cpp
)
target_link_libraries(tree_runner_exe PUBLIC my_custom_bt_nodes)

# Install executable
install(TARGETS tree_runner_exe
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
ament_export_dependencies(behaviortree_cpp rclcpp)

# Linting and testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
