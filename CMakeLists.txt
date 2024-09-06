cmake_minimum_required(VERSION 3.8)
project(serial_link_action_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Other dependencies
find_package(Eigen3 REQUIRED)
find_package(RobotLibrary REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
include_directories(${CMAKE_BINARY_DIR}/rosidl_generator_cpp)  # Ensure generated headers are included

# Message & Action Definitions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CartesianState.msg"
  "msg/CartesianTrajectoryPoint.msg"
  "msg/JointState.msg"
  "msg/JointTrajectoryPoint.msg"
  "msg/Statistics.msg"
  "action/TrackJointTrajectory.action"
  "action/TrackCartesianTrajectory.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs
)

# Velocity control server
add_executable(velocity_control_server src/velocity_control_server.cpp)

ament_target_dependencies(velocity_control_server
  "ament_cmake"
  "rclcpp"
  "rclcpp_action"
  "rosidl_default_runtime"
  "sensor_msgs"
  "std_msgs"
)

target_link_libraries(velocity_control_server
  ${PROJECT_NAME}__rosidl_typesupport_cpp  # Link the generated action typesupport
  Eigen3::Eigen
  RobotLibrary::RobotLibrary
)

rosidl_target_interfaces(velocity_control_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Demo client
add_executable(demo_client src/demo_client.cpp)

ament_target_dependencies(demo_client
  "ament_cmake"
  "rclcpp"
  "rclcpp_action"
  "rosidl_default_runtime"
  "std_msgs"
)

target_link_libraries(demo_client
  ${PROJECT_NAME}__rosidl_typesupport_cpp  # Link the generated action typesupport
  Eigen3::Eigen
  RobotLibrary::RobotLibrary
)

rosidl_target_interfaces(demo_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Installation
install(TARGETS
  velocity_control_server
  demo_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

