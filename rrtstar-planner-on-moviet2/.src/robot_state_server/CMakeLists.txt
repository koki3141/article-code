cmake_minimum_required(VERSION 3.8)
project(robot_state_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(robot_state_interface REQUIRED)

add_executable(sample_robot_state sample/sample_robot_state.cpp)
ament_target_dependencies(sample_robot_state
  moveit_ros_planning_interface
  rclcpp
)

add_executable(robot_state_server src/robot_state_server_node.cpp)
ament_target_dependencies(robot_state_server
  moveit_ros_planning_interface
  rclcpp
  robot_state_interface
)

install(TARGETS sample_robot_state robot_state_server
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
