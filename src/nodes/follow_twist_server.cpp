/**
 * @file    follow_twist_server.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   Demonstrates real-time Cartesian velocity control of a serial link robot.
 * 
 * @details This executable creates an action server for the FollowTwist action. The TrackJointTrajectory
 *          action enables the robot to be moved in to pre-defined configurations, then the FollowTwist
 *          action can be executed. Twist commands can be streamed to a predefined topic which are then
 *          used as a control input to the robot arm.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the SerialLinkBase class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include <RobotLibrary/Control/SerialKinematicControl.h>                                            // For serial link robots
#include <serial_link_action_server/follow_twist.hpp>                                               // Real-time velocity control of endpoint
#include <serial_link_action_server/model_updater.hpp>                                              // Joint state subscriber
#include <serial_link_action_server/track_joint_trajectory.hpp>                                     // Joint trajectory tracking
#include <serial_link_action_server/utilities.hpp>                                                  // Helper functions

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            MAIN                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    using namespace serial_link_action_server;
    
    rclcpp::init(argc, argv);                                                                       // Launches ROS2

    // Ensure sufficient number of arguments is provided
    if (argc < 5)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Invalid number of arguments. "
                     "Usage: <executable> <urdf_path> <endpoint_name> <control_topic_name> <joint_state_topic_name>");

        rclcpp::shutdown();
        
        return 1;
    }
 
    // For clarity:
    std::string urdfPath        = argv[1];
    std::string endpointName    = argv[2];
    std::string controlTopic    = argv[3];
    std::string jointStateTopic = argv[4];
        
    try 
    {
        auto model = std::make_shared<RobotLibrary::Model::KinematicTree>(urdfPath);                // Generate dynamic model
        
        auto modelUpdaterNode = std::make_shared<ModelUpdater>(model, jointStateTopic, endpointName); // Create node for updating joint state
        
        auto serverNode = std::make_shared<rclcpp::Node>(model->name()+"_action_server");           // Create action server nodes
        
        auto controller = std::make_shared<RobotLibrary::Control::SerialKinematicControl>(model, endpointName, load_control_parameters(serverNode));
          
        // Declare action servers
        auto mutex = std::make_shared<std::mutex>();                                                // This stops 2 actions using the robot at the same time

        TrackJointTrajectory jointTrajectoryServer(
            serverNode,
            controller,
            mutex,
            "track_joint_trajectory",
            controlTopic);
            
        FollowTwist follow_twist(
            serverNode,
            controller,
            mutex,
            "follow_twist",
            controlTopic);
                 
        // Add nodes to executor and spin
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(modelUpdaterNode);
        executor.add_node(serverNode);
        
        executor.spin();
        
        rclcpp::shutdown();
        
        return 0;

    }
    catch(const std::exception &exception)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), exception.what());
        
        rclcpp::shutdown();
        
        return 1;
    }  
}
