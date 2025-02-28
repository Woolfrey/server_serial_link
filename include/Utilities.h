/**
 * @file    Utilities.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   Useful functions for use in action servers.
 * 
 * @details This header file contains forward declarations of functions that are useful in action servers.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the SerialLinkBase class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */
 
#ifndef UTILITIES_H
#define UTILITIES_H
 
#include <Eigen/Core>
#include <geometry_msgs/msg/twist.hpp>
#include <RobotLibrary/Control/SerialLinkBase.h>
#include <rclcpp/rclcpp.hpp>
#include <serial_link_interfaces/msg/statistics.hpp>

/**
 * @brief Set control parameters for the action server.
 * @param controller A RobotLibrary::SerialLinkBase class (or derivative).
 * @return Returns false if at least one parameter cannot be set.
 */
RobotLibrary::Control::Options
get_control_parameters(const std::shared_ptr<rclcpp::Node> &node);

/**
 * @brief Updates min & max, and the mean and variance recursively
 * @param statistics A custom message that contains statistical data field
 * @param newValue The new computed value to use for the update
 * @param n The current sample size.
 */
void
update_statistics(serial_link_interfaces::msg::Statistics &statistics,
                  const double &newValue,
                  const unsigned int &n);

/**
 * @brief Puts an Eigen::Vector<double,6> object in to a ROS2 geometry_msgs/Twist
 * @param feedbackTwist The ROS2 object to put the data in to.
 * @param twist The Eigen::Vector object with the data we need.
 */
void
Eigen_twist_to_ROS(geometry_msgs::msg::Twist &feedbackTwist,
                   const Eigen::Vector<double, 6> &twist);

#endif
