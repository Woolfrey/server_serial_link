/**
 * @file    utilities.cpp
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
 
#include <serial_link_action_server/utilities.hpp>

namespace serial_link_action_server {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Convert a std::vector object to an Eigen::Matrix                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,6>
vector_to_matrix(const std::vector<double> vector)
{
    Eigen::Matrix<double,6,6> matrix;  // Value to be returned
    
    if(vector.size() != 36)
    {
        throw std::invalid_argument("Expected a vector with 36 elements, but it had " + std::to_string(vector.size()) + ".");
    }

    // Fill matrix in row-major order
    for (std::size_t i = 0; i < 6; ++i)
    {
        for (std::size_t j = 0; j < 6; ++j)
        {
            matrix(i, j) = vector[i * 6 + j];
        }
    }
    
    return matrix;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Get the control parameters from the ROS2 parameter server                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
RobotLibrary::Control::Parameters
load_control_parameters(const std::shared_ptr<rclcpp::Node> &node)
{
    RobotLibrary::Control::Parameters parameters;

    // Declare parameters for controller
    node->declare_parameter("cartesian_damping", std::vector<double>());
    node->declare_parameter("cartesian_stiffness", std::vector<double>());
    node->declare_parameter("frequency", parameters.controlFrequency);
    node->declare_parameter("joint_position_gain", parameters.jointPositionGain);
    node->declare_parameter("joint_velocity_gain", parameters.jointVelocityGain);
    node->declare_parameter("manipulability_threshold", parameters.minManipulability);
    node->declare_parameter("max_joint_acceleration", parameters.maxJointAcceleration);
    
    // Declare parameters for QP solver
    node->declare_parameter("barrier_reduction_rate", parameters.qpsolver.barrierReductionRate);
    node->declare_parameter("initial_barrier_scalar", parameters.qpsolver.initialBarrierScalar);
    node->declare_parameter("max_steps", static_cast<int>(parameters.qpsolver.maxSteps));           // Need to cast from unsigned int to int
    node->declare_parameter("step_size_tolerance", parameters.qpsolver.stepSizeTolerance);
    
    // Get control parameters
    std::vector<double> damping_vec;
    node->get_parameter("cartesian_damping", damping_vec);
    parameters.cartesianDamping = vector_to_matrix(damping_vec);

    std::vector<double> stiffness_vec;
    node->get_parameter("cartesian_stiffness", stiffness_vec);
    parameters.cartesianStiffness = vector_to_matrix(stiffness_vec);

    node->get_parameter("frequency", parameters.controlFrequency);
    node->get_parameter("joint_position_gain", parameters.jointPositionGain);
    node->get_parameter("joint_velocity_gain", parameters.jointVelocityGain);
    node->get_parameter("manipulability_threshold", parameters.minManipulability);
    node->get_parameter("max_joint_acceleration", parameters.maxJointAcceleration);

    // Get QP solver parameters
    node->get_parameter("barrier_reduction_rate", parameters.qpsolver.barrierReductionRate);
    node->get_parameter("initial_barrier_scalar", parameters.qpsolver.initialBarrierScalar);
    node->get_parameter("step_size_tolerance", parameters.qpsolver.stepSizeTolerance);
    node->get_parameter("max_steps", parameters.qpsolver.maxSteps);
    
    return parameters;
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

}
