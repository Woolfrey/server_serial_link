/**
 * @file   velocity_control_server.cpp
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  This is testing & demonstration.
 */

#include <ModelUpdater.h>                                                                           // Joint state subscriber
#include <RobotLibrary/Control/SerialKinematicControl.h>                                            // For serial link robots
#include <TrackCartesianTrajectory.h>
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

        auto modelUpdateNode = std::make_shared<ModelUpdate>(model);                                
        
        auto serverNode = std::make_shared<rclcpp::Node>(robotModel.name()+"_action_server");       // Create the server node
        
        auto controller = std::make_shared<RobotLibrary::Control::SerialKinematicControl>(model,
                                                                                          endpointName,
                                                                                          get_control_options(serverNode));

        auto mutex = std::make_shared<std::mutex>();                                               
        
        // List actions
        TrackJointTrajectory jointTrajectoryServer(serverNode,
                                                   controller,
                                                   mutex,
                                                   "track_joint_trajectory",
                                                   "joint_command_relay");
                                                
        TrackCartesianTrajectory cartesianTrajectoryServer(serverNode,
                                                           controller,
                                                           mutex,
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

