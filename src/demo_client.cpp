/**
 * @file   demo_client.cpp
 * @author Jon Woolfrey
 * @date   August 2024
 * @brief  This is for testing action servers.
 */

#include "JointTrajectoryClient.h"
#include <random>                                                                                   // For generating random numbers
#include "rclcpp/rclcpp.hpp"                                                                        // ROS2 C++ library
#include "rclcpp_action/rclcpp_action.hpp"                                                          // ROS2 C++ action library
#include <thread>                                                                                   // Threading (duh!)
#include "serial_link_action_server/action/track_joint_trajectory.hpp"                              // Custom action

using TrackJointTrajectory = serial_link_action_server::action::TrackJointTrajectory;

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

    std::thread spinThread(spin_node, clientNode);                                                  // Start spinning the node in a separate thread

    bool clientActive = true;                                                                       // Main loop control flag

    while (clientActive && rclcpp::ok())
    {
        RCLCPP_INFO(clientNode->get_logger(), "Enter command (close, home, random):");

        // Get user input
        std::string commandPrompt;
        std::getline(std::cin, commandPrompt);

        // Handle "close" command to shut down the client
        if (commandPrompt == "close")
        {
            RCLCPP_INFO(clientNode->get_logger(), "Shutting down.");
            clientActive = false;
        }
        // Handle joint trajectory commands ("home" or "random")
        else if (commandPrompt == "home" || commandPrompt == "random")
        {
            serial_link_action_server::msg::JointTrajectoryPoint endPoint;
            endPoint.time = 5.0;  // Set movement time

            // Populate joint positions
            if (commandPrompt == "home")
            {
                RCLCPP_INFO(clientNode->get_logger(), "Moving to home position.");
                endPoint.position.assign(numJoints, 0.0); // All joints to 0
            }
            else if (commandPrompt == "random")
            {
                RCLCPP_INFO(clientNode->get_logger(), "Moving to random joint position.");
                std::uniform_real_distribution<double> uniformDistribution(-2.0, 2.0);
                std::default_random_engine randomEngine(static_cast<unsigned>(std::time(0)));

                // Assign random positions
                endPoint.position.clear(); // Clear previous positions
                for (int i = 0; i < numJoints; ++i)
                {
                    endPoint.position.push_back(uniformDistribution(randomEngine));
                }
            }

            // Create goal and attach trajectory points
            auto goal = std::make_shared<TrackJointTrajectory::Goal>();
            goal->points = {endPoint};
            goal->delay = 0.0;

            jointTrajectoryClient.send_request(goal);
        }
        else
        {
            RCLCPP_WARN(clientNode->get_logger(), "Unknown command. Please enter 'close', 'home', or 'random'.");
        }
    }

    // Clean up and shutdown
    rclcpp::shutdown();
    spinThread.join(); // Wait for the spin thread to finish
    return 0;
}
