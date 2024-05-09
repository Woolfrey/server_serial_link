cmake_minimum_required(VERSION 3.8)
project(serial_link_action_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)                                                                        # Specifies location of header files

####################################################################################################
#                                         DEPENDENCIES                                             #
####################################################################################################

# ROS2:
find_package(ament_cmake REQUIRED)                                                                  # I don't know what this does ¯\_(ツ)_/¯ 
find_package(rclcpp REQUIRED)                                                                       # ROS2 C++ libraries
find_package(rclcpp_action REQUIRED)                                                                # ROS2 C++ Action libraries
find_package(serial_link_interfaces REQUIRED)                                                       # Previously built ROS2 package

# RobotLibrary:
find_package(RobotLibrary REQUIRED)                                                                 # Custom classes for robot modeling & control
find_package(Eigen3 REQUIRED)                                                                       # RobotLibrary relies on Eigen

####################################################################################################
#                                  MESSAGE & ACTION DEFINITIONS                                    #        
####################################################################################################
add_executable(action_test test/test.cpp)
ament_target_dependencies(action_test "ament_cmake" "rclcpp" "rclcpp_action" "serial_link_interfaces")
target_link_libraries(action_test RobotLibrary::RobotLibrary Eigen3::Eigen)
   
# This is so ROS2 can find the package                                               
install(TARGETS
        action_test
        DESTINATION lib/${PROJECT_NAME})

ament_package()
