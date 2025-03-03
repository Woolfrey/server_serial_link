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
RobotLibrary::Control::Options
get_control_parameters(const std::shared_ptr<rclcpp::Node> &node)
{
    RobotLibrary::Control::Options options;                                                         // We want to return this
    
    // We need to declare all the parameters before we can get them.
    node->declare_parameter("cartesian_damping", std::vector<double>());                            // Gain on endpoint velocity error
    node->declare_parameter("cartesian_stiffness", std::vector<double>());                          // Gain on endpoint pose error
    node->declare_parameter<double>("joint_position_gain", 50.0);                                   // Gain on joint position error
    node->declare_parameter<double>("joint_velocity_gain", 1.0);                                    // Gain on joint velocity error
    node->declare_parameter<double>("manipulability_threshold", 1e-03);                             // For singularity avoidance
    node->declare_parameter<double>("solver_initial_barrier", 100.0);                               // Initial scalar for log barrier function in QP solver
    node->declare_parameter<double>("solver_reduction_rate", 1e-02);                                // Multiplier for log barrier in QP solver
    node->declare_parameter<int>("max_steps", 5);                                                   // Maximum iterations for the solver algorithm 

    std::vector<double> temp;                                                                       // Temporary storage
    
    // Get & store damping
    temp = node->get_parameter("cartesian_damping").as_double_array();
    
    if(temp.size() != 36)
    {
        throw std::invalid_argument("Cartesian damping must have exactly 36 elements, but had " + std::to_string(temp.size()) + ".");
    }
    
    for(int i = 0; i < 6; ++i) for(int j = 0; j < 6; ++j) options.cartesianDamping(i,j) = temp[i*6+j];
    
    // Get & store stiffness
    temp = node->get_parameter("cartesian_stiffness").as_double_array();
    
    if(temp.size() != 36)
    {
        throw std::invalid_argument("Cartesian stiffness must have exactly 36 elements, but had " + std::to_string(temp.size()) + ".");
    }
    
    for(int i = 0; i < 6; ++i) for(int j = 0; j < 6; ++j) options.cartesianStiffness(i,j) = temp[i*6+j];
   
    // Load other parameters
    options.jointPositionGain = node->get_parameter("joint_position_gain").as_double();
    options.jointVelocityGain = node->get_parameter("joint_velocity_gain").as_double();
    options.minManipulability = node->get_parameter("manipulability_threshold").as_double();
    options.qpsolver.initialBarrierScalar = node->get_parameter("solver_initial_barrier").as_double();
    options.qpsolver.barrierReductionRate = node->get_parameter("solver_reduction_rate").as_double();
    options.qpsolver.maxSteps = node->get_parameter("max_steps").as_int();
   
    return options;
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
