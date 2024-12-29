/**
 * @file   demo_client.cpp
 * @author Jon Woolfrey
 * @date   August 2024
 * @brief  This is for testing action servers.
 */

#include "client/CartesianTrajectoryClient.h"
#include "client/JointTrajectoryClient.h"
#include <random>                                                                                   // For generating random numbers
#include "rclcpp/rclcpp.hpp"                                                                        // ROS2 C++ library
#include "rclcpp_action/rclcpp_action.hpp"                                                          // ROS2 C++ action library
#include <thread>                                                                                   // Threading (duh!)
#include "Utilities.h"                                                                              // Useful functions

using JointTrajectoryPoint     = serial_link_action_server::msg::JointTrajectoryPoint;
using CartesianTrajectoryPoint = serial_link_action_server::msg::CartesianTrajectoryPoint;
using TrackCartesianTrajectory = serial_link_action_server::action::TrackCartesianTrajectory;
using TrackJointTrajectory     = serial_link_action_server::action::TrackJointTrajectory;
  /////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          MAIN                                                   //
/////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);                                                                       // Starts up ROS2
   
    auto clientNode = rclcpp::Node::make_shared("serial_link_demo_client");                         // Create client node and advertise its name
    auto jointConfigurations = load_joint_configurations();                                         // From the parameter server
    auto endpointPoses = load_endpoint_poses();                                                     // From the parameter server

    // Create the action clients, attach to node
    JointTrajectoryClient jointTrajectoryClient(clientNode, "track_joint_trajectory", true);
    CartesianTrajectoryClient cartesianTrajectoryClient(clientNode, "track_cartesian_trajectory", true);  
    ActionClientInterface *activeClient = nullptr;                                                  // Use this to keep track of which action is running           
   
    std::thread{[clientNode]() { rclcpp::spin(clientNode); }}.detach();                             // Spin the node in a separate thread so we can continue
  
    while (rclcpp::ok())
    {
        // Get user input
        RCLCPP_INFO(clientNode->get_logger(), "Enter a command. Type 'options' to a see a list.");
        std::string commandPrompt;
        std::getline(std::cin, commandPrompt);
        
        if (commandPrompt == "options")
        {
            RCLCPP_INFO(clientNode->get_logger(), "Available joint commands:");
            for (const auto &config : jointConfigurations)
            {
                RCLCPP_INFO(clientNode->get_logger(), "- %s", config.first.c_str());
            }
            RCLCPP_INFO(clientNode->get_logger(), "Available Cartesian commands:");
            for(const auto &pose : endpointPoses)
            {
                RCLCPP_INFO(clientNode->get_logger(), "- %s", pose.first.c_str());
            }
        }
        else if (commandPrompt == "close")
        {
            RCLCPP_INFO(clientNode->get_logger(), "Shutting down.");
            
            stop_robot(activeClient);

            break;
        }
        else if(commandPrompt == "cancel" or  commandPrompt == "")
        {
            stop_robot(activeClient);
        }
        else
        {
            auto iterator = jointConfigurations.find(commandPrompt);
            
            if (iterator != jointConfigurations.end())
            {   
                if (activeClient != nullptr and activeClient->is_running())
                {
                    stop_robot(activeClient);
                }
                
                auto goal = std::make_shared<TrackJointTrajectory::Goal>();                         // Generate goal object
                
                goal->points = iterator->second;                                                    // Attach the joint trajectory
                
                RCLCPP_INFO(clientNode->get_logger(), "Moving to `%s` configuration(s).", commandPrompt.c_str()); // Inform user

                jointTrajectoryClient.send_request(goal);                                           // Send request to client
                
                activeClient = &jointTrajectoryClient;                                             
            }
            else
            {
                auto iterator = endpointPoses.find(commandPrompt);
                
                if (iterator != endpointPoses.end())
                {   
                    if (activeClient != nullptr and activeClient->is_running())
                    {
                        stop_robot(activeClient);
                    }
                    
                    auto goal = std::make_shared<TrackCartesianTrajectory::Goal>();                 // Generate goal object
                    
                    goal->points = iterator->second;                                                // Attach the joint trajectory
                    
                    RCLCPP_INFO(clientNode->get_logger(), "Moving `%s` .", commandPrompt.c_str());  // Inform user

                    cartesianTrajectoryClient.send_request(goal);                                   // Send request to client
                    
                    activeClient = &cartesianTrajectoryClient;                                             
                }
                else
                {
                    RCLCPP_WARN(clientNode->get_logger(), "Unknown command. Type 'options' to see a list.");
                }
            }
        }
    }

    rclcpp::shutdown();                                                                             // Shut down

    return 0;
}
