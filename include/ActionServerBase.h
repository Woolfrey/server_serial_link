#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ libraries
#include <rclcpp_action/rclcpp_action.hpp>                                                          // ROS2 action libaries
#include <serial_link_interfaces/action/move_to_joint_position.hpp>                                 // Custom action built in another project
#include <serial_link_interfaces/action/move_to_joint_positions.hpp>                                // Custom action built in another project

// Short definitions for actions
using JointPositionAction      = serial_link_interfaces::action::MoveToJointPosition;
using MultiJointPositionAction = serial_link_interfaces::action::MoveToJointPositions;

// Short definitions for goal handler
using JointPositionManager      = rclcpp_action::ServerGoalHandle<JointPositionAction>;
using MultiJointPositionManager = rclcpp_action::ServerGoalHandle<MultiJointPositionAction>;
