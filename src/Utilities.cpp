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
