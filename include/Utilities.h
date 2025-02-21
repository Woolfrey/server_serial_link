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
 
#include <RobotLibrary/Control/SerialLinkBase.h>

/**
 * @brief Set control parameters for the action server.
 * @param controller A RobotLibrary::SerialLinkBase class (or derivative).
 * @return Returns false if at least one parameter cannot be set.
 */
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
