cmake_minimum_required(VERSION 3.5)
project(robot_status_monitor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1. 設定 msg 檔案
set(msg_files
  "msg/RobotStatus.msg"
)

# 2. 生成 ROS 2 介面 (msg)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

# 3. 設定包含目錄
include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

# 4. 建立 Publisher
add_executable(robot_status_publisher src/robot_status_publisher.cpp)
ament_target_dependencies(robot_status_publisher rclcpp std_msgs)

# 5. 建立 Subscriber
add_executable(robot_status_subscriber src/robot_status_subscriber.cpp)
ament_target_dependencies(robot_status_subscriber rclcpp std_msgs)

# 6. 讓執行檔依賴已生成的介面
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(robot_status_publisher ${cpp_typesupport_target})
target_link_libraries(robot_status_subscriber ${cpp_typesupport_target})

# 7. 安裝執行檔
install(TARGETS
  robot_status_publisher
  robot_status_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
