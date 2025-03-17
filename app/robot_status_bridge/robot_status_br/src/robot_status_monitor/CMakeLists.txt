cmake_minimum_required(VERSION 3.5)
project(robot_status_monitor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Set msg files
set(msg_files
  "msg/RobotStatus.msg"
)

# Generate ROS 2 interfaces (msg)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

# Set include directories
include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

add_executable(robot_status_publisher src/robot_status_publisher.cpp)
ament_target_dependencies(robot_status_publisher rclcpp std_msgs)
add_executable(robot_status_subscriber src/robot_status_subscriber.cpp)
ament_target_dependencies(robot_status_subscriber rclcpp std_msgs)
 
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(robot_status_publisher ${cpp_typesupport_target})
target_link_libraries(robot_status_subscriber ${cpp_typesupport_target})

install(TARGETS
  robot_status_publisher
  robot_status_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
