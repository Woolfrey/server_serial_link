/**
 * @file    utilities.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    August 2025
 * @version 2.0
 *
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
RobotLibrary::Control::SerialLinkParameters
load_control_parameters(const std::shared_ptr<rclcpp::Node> &node)
{
    RobotLibrary::Control::SerialLinkParameters parameters;

    // RobotLibrary specific:
    parameters.cartesianVelocityGain = vector_to_matrix(node->declare_parameter<std::vector<double>>("cartesian_velocity_gain", std::vector<double>{}));
    parameters.cartesianPoseGain     = vector_to_matrix(node->declare_parameter<std::vector<double>>("cartesian_pose_gain", std::vector<double>{}));
    parameters.controlFrequency      = node->declare_parameter<double>("frequency", parameters.controlFrequency);
    parameters.jointPositionGains    = node->declare_parameter<std::vector<double>>("joint_position_gains", std::vector<double>{});
    parameters.jointVelocityGains    = node->declare_parameter<std::vector<double>>("joint_velocity_gains", std::vector<double>{});
    parameters.minManipulability     = node->declare_parameter<double>("manipulability_threshold", parameters.minManipulability);
    parameters.maxJointAcceleration  = node->declare_parameter<double>("max_joint_acceleration", parameters.maxJointAcceleration);

    // For the QP solver:
    parameters.qpsolver.barrierReductionRate = node->declare_parameter<double>("barrier_reduction_rate", parameters.qpsolver.barrierReductionRate);
    parameters.qpsolver.initialBarrierScalar = node->declare_parameter<double>("initial_barrier_scalar", parameters.qpsolver.initialBarrierScalar);
    parameters.qpsolver.maxSteps             = static_cast<unsigned int>(node->declare_parameter<int>("max_steps", static_cast<int>(parameters.qpsolver.maxSteps)));
    parameters.qpsolver.stepSizeTolerance    = node->declare_parameter<double>("step_size_tolerance", parameters.qpsolver.stepSizeTolerance);

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

} // namespace
