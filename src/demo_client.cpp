/**
 * @file   demo_client.cpp
 * @author Jon Woolfrey
 * @date   August 2024
 * @brief  This is for testing action servers.
 */

#include "CartesianTrajectoryClient.h"
#include "JointTrajectoryClient.h"
#include <random>                                                                                   // For generating random numbers
#include "rclcpp/rclcpp.hpp"                                                                        // ROS2 C++ library
#include "rclcpp_action/rclcpp_action.hpp"                                                          // ROS2 C++ action library
#include <thread>                                                                                   // Threading (duh!)

using TrackCartesianTrajectory = serial_link_action_server::action::TrackCartesianTrajectory;
using TrackJointTrajectory     = serial_link_action_server::action::TrackJointTrajectory;

/**
 * This thread spins the client node indefinitely.
 * @param node The thing we want to run in the background.
 */
void spin_node(rclcpp::Node::SharedPtr node)
{
    rclcpp::spin(node);
}

  /////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          MAIN                                                   //
/////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    if (argc != 2) throw std::invalid_argument("Require number of joints as input.");

    int numJoints = std::stoi(argv[1]);
    rclcpp::init(argc, argv);

    // Create the client node and clients
    auto clientNode = rclcpp::Node::make_shared("serial_link_demo_client");
    
    JointTrajectoryClient jointTrajectoryClient(clientNode, "track_joint_trajectory");
    
    CartesianTrajectoryClient cartesianTrajectoryClient(clientNode, "track_cartesian_trajectory");

    std::thread spinThread(spin_node, clientNode);                                                  // Start spinning the node in a separate thread

    bool clientActive = true;                                                                       // Main loop control flag

    while (clientActive && rclcpp::ok())
    {
    
        // Get user input
        RCLCPP_INFO(clientNode->get_logger(), "Enter command (close, home, random).");
        std::string commandPrompt;
        std::getline(std::cin, commandPrompt);

        // Handle "close" command to shut down the client
        if (commandPrompt == "close")
        {
            RCLCPP_INFO(clientNode->get_logger(), "Shutting down.");
            clientActive = false;
            
            jointTrajectoryClient.cancel_action();
            cartesianTrajectoryClient.cancel_action();
        }
        else if(commandPrompt == "cancel" or  commandPrompt == "")
        {
            jointTrajectoryClient.cancel_action();
            cartesianTrajectoryClient.cancel_action();
        }
        else if (commandPrompt == "home" or commandPrompt == "random")
        {
            // Check if action is currently running. Cancel if true.
            if(jointTrajectoryClient.action_is_running())
            {
                jointTrajectoryClient.cancel_action();
            }
            else if(cartesianTrajectoryClient.action_is_running())
            {
                cartesianTrajectoryClient.cancel_action();
            }
            
            serial_link_action_server::msg::JointTrajectoryPoint endPoint;                          // Final state for the trajectory
            
            endPoint.time = 5.0;                                                                    // Set movement time

            // Populate joint positions
            if (commandPrompt == "home")
            {
                RCLCPP_INFO(clientNode->get_logger(), "Moving to home position.");
                endPoint.position.assign(numJoints, 0.0);                                           // All joints to 0
            }
            else if (commandPrompt == "random")
            {
                RCLCPP_INFO(clientNode->get_logger(), "Moving to random joint position.");
                std::uniform_real_distribution<double> uniformDistribution(-2.0, 2.0);
                std::default_random_engine randomEngine(static_cast<unsigned>(std::time(0)));

                // Assign random positions
                endPoint.position.clear();                                                          // Clear previous positions
                for (int i = 0; i < numJoints; ++i)
                {
                    endPoint.position.push_back(uniformDistribution(randomEngine));
                }
            }

            // Create goal and attach trajectory points
            auto goal = std::make_shared<TrackJointTrajectory::Goal>();
            goal->points = {endPoint};

            jointTrajectoryClient.send_request(goal);
        }
        // Handle cartesian trajectory commands
        else if (commandPrompt == "left" or commandPrompt == "right" or
                 commandPrompt == "up"   or commandPrompt == "down"  or
                 commandPrompt == "fore" or commandPrompt == "aft"   or
                 commandPrompt == "hold")
        {
        
            // Check if action is currently running. Cancel if true.
            if(jointTrajectoryClient.action_is_running())
            {
                jointTrajectoryClient.cancel_action();
            }
            else if(cartesianTrajectoryClient.action_is_running())
            {
                cartesianTrajectoryClient.cancel_action();
            }
            
            RCLCPP_INFO(clientNode->get_logger(), "Moving endpoint %s.", commandPrompt.c_str());

            // Set up cartesian trajectory point
            serial_link_action_server::msg::CartesianTrajectoryPoint endPoint;
            endPoint.time = 2.0;
            endPoint.reference = 2;                                                                 // Relative to endpoint frame

            // Update endpoint position based on the command
                 if (commandPrompt == "left")  endPoint.pose.position.y =  0.2;
            else if (commandPrompt == "right") endPoint.pose.position.y = -0.2;
            else if (commandPrompt == "up")    endPoint.pose.position.z =  0.2;
            else if (commandPrompt == "down")  endPoint.pose.position.z = -0.2;
            else if (commandPrompt == "fore")  endPoint.pose.position.x =  0.2;
            else if (commandPrompt == "aft")   endPoint.pose.position.x = -0.2;
            else if (commandPrompt == "hold")  endPoint.time = 10.0;

            // No orientation change
            endPoint.pose.orientation.w = 1.0;
            endPoint.pose.orientation.x = 0.0;
            endPoint.pose.orientation.y = 0.0;
            endPoint.pose.orientation.z = 0.0;

            // Create goal and attach cartesian point
            auto goal = std::make_shared<TrackCartesianTrajectory::Goal>();
            goal->points = {endPoint};                                                              // This turns it in to an array with 1 element
            
            cartesianTrajectoryClient.send_request(goal);
        }
        else
        {
            RCLCPP_WARN(clientNode->get_logger(), "Unknown command. Please enter 'close', 'home', or 'random'.");
        }
    }

    // Clean up and shutdown
    rclcpp::shutdown();
    spinThread.join();                                                                              // Wait for the spin thread to finish
    return 0;
}
