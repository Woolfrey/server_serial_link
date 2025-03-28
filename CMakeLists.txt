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
find_package(sensor_msgs REQUIRED)
find_package(serial_link_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# 3rd party packages
find_package(Eigen3 REQUIRED)
find_package(RobotLibrary REQUIRED)

# Include directories
include_directories(include
                    ${EIGEN3_INCLUDE_DIRS}
                    ${RobotLibrary_INCLUDE_DIRS})

# Trajectory tracking server
add_executable(trajectory_tracking_server src/nodes/trajectory_tracking_server.cpp
                                          src/model_updater.cpp
                                          src/track_cartesian_trajectory.cpp
                                          src/track_joint_trajectory.cpp
                                          src/utilities.cpp)
                                          
target_link_libraries(trajectory_tracking_server Eigen3::Eigen RobotLibrary::RobotLibrary)

ament_target_dependencies(trajectory_tracking_server "ament_cmake"
                                                     "geometry_msgs"
                                                     "rclcpp"
                                                     "rclcpp_action"
                                                     "sensor_msgs"
                                                     "serial_link_interfaces"
                                                     "std_msgs")
# Installation
install(TARGETS trajectory_tracking_server
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
