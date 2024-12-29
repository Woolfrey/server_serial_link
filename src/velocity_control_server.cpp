/**
 * @file   velocity_control_server.cpp
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  This is testing & demonstration.
 */

#include <ModelUpdater.h>                                                                           // Joint state subscriber
#include <RobotLibrary/SerialKinematicControl.h>                                                    // For serial link robots
#include <server/TrackCartesianTrajectory.h>
#include <server/TrackJointTrajectory.h>
#include "Utilities.h"

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            MAIN                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                                                                       // Launches ROS2

    try
    {
        auto paramNode = std::make_shared<rclcpp::Node>("param_loader");                            // Temporary node to load parameters

        // Load all parameters at once
        std::string urdfLocation = paramNode->declare_parameter<std::string>("urdf", "");
        double frequency = paramNode->declare_parameter<double>("frequency", 500.0);
        std::string endpointName = paramNode->declare_parameter<std::string>("endpoint", "unnamed");  
        std::string controlTopicName = paramNode->declare_parameter<std::string>("control_topic", "joint_commands");
       
        paramNode.reset();                                                                          // Free the node and its resources

        // Create model & controller
        RobotLibrary::KinematicTree robotModel(urdfLocation);                                       // Create the robot model
        RobotLibrary::SerialKinematicControl controller(&robotModel, endpointName, frequency);      // Create controller, attach model
        
        if(not set_control_parameters(controller))
        {
            throw std::runtime_error("Failed to set control parameters for some reason.");
        }
          
        // Create nodes
        auto modelUpdaterNode = std::make_shared<ModelUpdater>(&robotModel);                        // Reads & updates joint state
        auto serverNode = std::make_shared<rclcpp::Node>(robotModel.name() + "_action_server");     // Create server node

        // List actions, attach server node      
        std::mutex mutex;                                                                           // Stops 2 actions using robot simultaneously
               
        TrackJointTrajectory jointTrajectoryServer(
            serverNode,
            &controller,
            &mutex,
            "track_joint_trajectory",
            "joint_command_relay");
                                                   
        TrackCartesianTrajectory cartesianTrajectoryServer(
            serverNode,
            &controller,
            &mutex,
            "track_cartesian_trajectory",
            "joint_command_relay");

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

