cmake_minimum_required(VERSION 3.8)
project(serial_link_action_server)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler options for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 packages
find_package(action_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 3rd party packages
find_package(Eigen3 REQUIRED)
find_package(RobotLibrary REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${CMAKE_BINARY_DIR}/rosidl_generator_cpp  # Ensure generated headers are included
)

# Message & Action Definitions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CartesianState.msg"
  "msg/CartesianTrajectoryPoint.msg"
  "msg/JointCommand.msg"
  "msg/JointState.msg"
  "msg/JointTrajectoryPoint.msg"
  "msg/Statistics.msg"
  "action/TrackJointTrajectory.action"
  "action/TrackCartesianTrajectory.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs
)

# Velocity Control Server
add_executable(velocity_control_server src/velocity_control_server.cpp)
ament_target_dependencies(velocity_control_server
  "ament_cmake"
  "geometry_msgs"
  "rclcpp"
  "rclcpp_action"
  "rosidl_default_runtime"
  "sensor_msgs"
  "std_msgs"
)
rosidl_get_typesupport_target(${PROJECT_NAME}__rosidl_typesupport_cpp ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(velocity_control_server
  ${PROJECT_NAME}__rosidl_typesupport_cpp  # Link the generated action typesupport
  Eigen3::Eigen
  RobotLibrary::RobotLibrary
)

# Demo Client
add_executable(demo_client src/demo_client.cpp)
ament_target_dependencies(demo_client
  "action_msgs"
  "ament_cmake"
  "rclcpp"
  "rclcpp_action"
  "rosidl_default_runtime"
  "std_msgs"
)
target_link_libraries(demo_client
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  Eigen3::Eigen
  RobotLibrary::RobotLibrary
)

# Mujoco Relay
add_executable(mujoco_relay src/mujoco_relay.cpp)
ament_target_dependencies(mujoco_relay
  "ament_cmake"
  "rclcpp"
  "rosidl_default_runtime"
  "std_msgs"
)
target_link_libraries(mujoco_relay
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  Eigen3::Eigen
  RobotLibrary::RobotLibrary
)

# Installation
install(TARGETS velocity_control_server
                demo_client
                mujoco_relay
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
                  config
                  test
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
