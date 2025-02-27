/**
 * @file    Utilities.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   Useful functions for use in action servers.
 * 
 * @details This source file elaborates on the functions in include/Utilities.h
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the SerialLinkBase class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */
 
 
#include <Utilities.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Set the control gains and other parameters                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool
set_control_parameters(RobotLibrary::Control::SerialLinkBase &controller)
{
    auto configNode = rclcpp::Node::make_shared("control_parameters");

    // Load Cartesian damping matrix
    configNode->declare_parameter<std::vector<double>>("cartesian_damping", {}, rcl_interfaces::msg::ParameterDescriptor{});
    std::vector<double> damping = configNode->get_parameter("cartesian_damping").as_double_array();

    if (damping.size() != 36)
    {
        RCLCPP_ERROR(configNode->get_logger(), "Cartesian damping must have exactly 36 elements.");
        return false;
    }

    Eigen::Matrix<double, 6, 6> cartesianDamping;
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 6; ++j) {
            cartesianDamping(i, j) = damping[i * 6 + j];
        }
    }

    // Load Cartesian stiffness matrix
    configNode->declare_parameter<std::vector<double>>("cartesian_stiffness", {}, rcl_interfaces::msg::ParameterDescriptor{});
    std::vector<double> stiffness = configNode->get_parameter("cartesian_stiffness").as_double_array();

    if (stiffness.size() != 36) {
        RCLCPP_ERROR(configNode->get_logger(), "Cartesian stiffness must have exactly 36 elements.");
        return false;
    }

    Eigen::Matrix<double, 6, 6> cartesianStiffness;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            cartesianStiffness(i, j) = stiffness[i * 6 + j];
        }
    }

    // Load scalar parameters
    double jointPositionGain = configNode->declare_parameter<double>("joint_position_gain", 50.0, rcl_interfaces::msg::ParameterDescriptor{});
    double jointVelocityGain = configNode->declare_parameter<double>("joint_velocity_gain", 1.0, rcl_interfaces::msg::ParameterDescriptor{});
    double manipulabilityThreshold = configNode->declare_parameter<double>("manipulability_threshold", 0.001, rcl_interfaces::msg::ParameterDescriptor{});

    // Set control parameters in the controller
    controller.set_cartesian_gains(cartesianStiffness, cartesianDamping);
    controller.set_joint_gains(jointPositionGain, jointVelocityGain);
    controller.set_manipulability_threshold(manipulabilityThreshold);

    RCLCPP_INFO(configNode->get_logger(), "Control parameters successfully loaded and set.");
    return true;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Recursively update statistics for certain result messages                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
update_statistics(serial_link_interfaces::msg::Statistics &statistics,
                  const double &newValue,
                  const unsigned int &n)
{
    statistics.min = std::min(statistics.min, newValue);
    statistics.max = std::max(statistics.max, newValue);
    
    if(n > 1)
    {
        statistics.mean = ((n-1) * statistics.mean + newValue) / n;
        
        double delta = newValue - statistics.mean;
        
        statistics.variance = ((n-2) * statistics.variance + (n / (n - 1)) * delta * delta ) / (n - 1);
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //              Put an Eigen::Vector<double,6> in to a geometry_msgs::msg::Twist                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
Eigen_twist_to_ROS(geometry_msgs::msg::Twist &feedbackTwist,
                   const Eigen::Vector<double, 6> &twist)
{
    feedbackTwist.linear.x  = twist[0];
    feedbackTwist.linear.y  = twist[1];
    feedbackTwist.linear.z  = twist[2];
    feedbackTwist.angular.x = twist[3];
    feedbackTwist.angular.y = twist[4];
    feedbackTwist.angular.z = twist[5];
}
