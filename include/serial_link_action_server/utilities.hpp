/**
 * @file    utilities.hpp
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
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <RobotLibrary/Control/SerialLinkBase.h>
#include <serial_link_interfaces/msg/statistics.hpp>

namespace serial_link_action_server {

/**
 * @brief Converts a std::vector object (with 36 elements) to a 6x6 Eigen::Matrix object.
 * @param vector The vector to be converted.
 * @return An Eigen::Matrix object with pre-defined dimensions.
 */
Eigen::Matrix<double,6,6>
vector_to_matrix(const std::vector<double> vector);

/**
 * @brief Loads parameters from ROS2 server and put them in to a RobotLibrary struct for a controller class.
 * @param node A ROS2 node class is required to retrieve parameters.
 * @return A custom struct which is used by the RobotLibrary::Control::SerialLinkBase class(es).
 */
RobotLibrary::Control::Parameters
load_control_parameters(const std::shared_ptr<rclcpp::Node> &node);

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
                   
/**
 * @brief Puts a RobotLibrary::Pose object in to a ROS2 geometry_msgs/Pose
 * @param feedbackPose The ROS msg
 * @param pose The RobotLibrary object.
 */
void
RL_pose_to_ROS(geometry_msgs::msg::Pose &feedbackPose,
               const RobotLibrary::Model::Pose &pose);
}

#endif
