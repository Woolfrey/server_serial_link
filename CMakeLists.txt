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
find_package(sensor_msgs REQUIRED)                                                                  # ROS2 sensor message definitions
find_package(serial_link_interfaces REQUIRED)                                                       # Previously built ROS2 package
find_package(std_msgs REQUIRED)

# RobotLibrary:
find_package(RobotLibrary REQUIRED)                                                                 # Custom classes for robot modeling & control
find_package(Eigen3 REQUIRED)                                                                       # RobotLibrary relies on Eigen

include_directories(${EIGEN3_INCLUDE_DIRS})

####################################################################################################
#                                  MESSAGE & ACTION DEFINITIONS                                    #        
####################################################################################################

########################################### Server #################################################
add_executable(velocity_control_server src/velocity_control_server.cpp)
               
ament_target_dependencies(velocity_control_server
                          "ament_cmake"
                          "rclcpp"
                          "rclcpp_action"
                          "sensor_msgs"
                          "serial_link_interfaces"
                          "std_msgs")
                          
target_link_libraries(velocity_control_server Eigen3::Eigen RobotLibrary::RobotLibrary)

######################################## Client ####################################################                        
add_executable(test_client src/test_client.cpp)
               
ament_target_dependencies(test_client
                          "ament_cmake"
                          "rclcpp"
                          "rclcpp_action"
                          "serial_link_interfaces"
                          "std_msgs")
                          
target_link_libraries(test_client Eigen3::Eigen RobotLibrary::RobotLibrary)

# This is so ROS2 can find the package                                               
install(TARGETS
        velocity_control_server
        test_client
        DESTINATION lib/${PROJECT_NAME})
        
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
