/**
 * @file    follow_twist_server.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   Creates an action server to enable a robot arm to follow a twist command.
 * 
 * @details This executable launches a trajectory tracking server, and server for following a twist command.
 *          The trajectory tracking server is used to move the robot in to pre-defined configurations.
 *          The follow twist action will subscribe to an advertised topic and execute the twist command.
 *
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the control class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */
 

#include <FollowTwist.h>
#include <ModelUpdater.h>                                                                           // Joint state subscriber
#include <RobotLibrary/Control/SerialKinematicControl.h>                                            // For serial link robots
#include <TrackJointTrajectory.h>
#include <Utilities.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            MAIN                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                                                                       // Launches ROS2
    
    // Load parameters (NOTE TO SELF: Need to clean this up!)
    auto paramNode = std::make_shared<rclcpp::Node>("param_loader");
    std::string urdfLocation = paramNode->declare_parameter<std::string>("urdf", "");
    std::string endpointName = paramNode->declare_parameter<std::string>("endpoint", "unnamed");  
    std::string controlTopicName = paramNode->declare_parameter<std::string>("control_topic", "joint_commands");
    paramNode.reset();                                                                              // Free the node and its resources
    
    try
    {
        auto model = std::make_shared<RobotLibrary::Model::KinematicTree>(urdfLocation);            // Create the model

        auto modelUpdaterNode = std::make_shared<ModelUpdater>(model);                                
        
        auto serverNode = std::make_shared<rclcpp::Node>(model->name()+"_action_server");            // Create the server node
        
        auto controller = std::make_shared<RobotLibrary::Control::SerialKinematicControl>(model,
                                                                                          endpointName,
                                                                                          get_control_parameters(serverNode));

        auto mutex = std::make_shared<std::mutex>();                                               
        
        // List actions, attach server node      
               
        TrackJointTrajectory jointTrajectoryServer(serverNode,
                                                   controller,
                                                   mutex,
                                                   "track_joint_trajectory",
                                                   "joint_command_relay");
                                                   
        FollowTwist followTwistServer(serverNode,
                                      controller,
                                      mutex,
                                      "follow_twist",
                                      "joint_command_relay",
                                      "twist_command");

        // Add nodes to executor & spin
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(modelUpdaterNode);                                                        // Add model updater node
        executor.add_node(serverNode);                                                              // Add server node

        executor.spin();                                                                            // Run the node(s)

        rclcpp::shutdown();                                                                         // As it says
        
        return 0;                                                                                   // No errors
    }
    catch (const std::exception &exception)
    {
        std::cerr << exception.what() << "\n";
        
        return -1;                                                                                  // Error occurred
    }
}

