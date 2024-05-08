cmake_minimum_required(VERSION 3.8)
project(serial_link_action_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################################################################################################
#                                         DEPENDENCIES                                             #
####################################################################################################
find_package(ament_cmake REQUIRED)                                                                  # I don't know what this does ¯\_(ツ)_/¯ 
find_package(rclcpp REQUIRED)                                                                       # ROS2 C++ libraries
find_package(rclcpp_action REQUIRED)                                                                # ROS2 C++ Action libraries
find_package(RobotLibrary REQUIRED)
find_package(serial_link_interfaces REQUIRED)                                                       # Previously built ROS2 package

include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

####################################################################################################
#                                  MESSAGE & ACTION DEFINITIONS                                    #        
####################################################################################################
add_executable(action_test test/test.cpp)
ament_target_dependencies(action_test
                          "rclcpp"
                          "rclcpp_action"
                          "serial_link_interfaces")
   
# This is so ROS2 can find the package                                               
install(TARGETS
        action_test
        DESTINATION lib/${PROJECT_NAME})

ament_package()
