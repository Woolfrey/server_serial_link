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

using TrackCartesianTrajectory = serial_link_action_server::action::TrackCartesianTrajectory;
using TrackJointTrajectory     = serial_link_action_server::action::TrackJointTrajectory;


bool stop_robot(ActionClientInterface *activeClient)
{
    if (activeClient == nullptr)
    {
        return true;                                                                                // No active action
    }

    // Possible return codes:
    // 0 = Unknown
    // 1 = Accepted
    // 2 = Executing
    // 3 = Canceling
    // 4 = Succeeded
    // 5 = Canceled
    // 6 = Aborted

    if (activeClient->status() == 0)
    {
        return false; // A problem
    }

    // If running, cancel
    if (activeClient->status() == 1 || activeClient->status() == 2)
    {
        activeClient->cancel_action();
    }

    // Wait until action is resolved
    while (activeClient->status() != 4 &&
           activeClient->status() != 5 &&
           activeClient->status() != 6)
    {      
        std::this_thread::sleep_for(std::chrono::milliseconds(10));                                 // Wait for 100ms
    }

    return true;
}

  /////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          MAIN                                                   //
/////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    if (argc != 2) throw std::invalid_argument("Require number of joints as input.");

    int numJoints = std::stoi(argv[1]);                                                             // Transfer
    
    rclcpp::init(argc, argv);                                                                       // Starts up ROS2

    auto clientNode = rclcpp::Node::make_shared("serial_link_demo_client");                         // Create client node and advertise its name
 
    JointTrajectoryClient jointTrajectoryClient(clientNode, "track_joint_trajectory");
    
    CartesianTrajectoryClient cartesianTrajectoryClient(clientNode, "track_cartesian_trajectory");
    
    ActionClientInterface *activeClient = nullptr;                                                  // Use this to keep track of which action is running           
    
    std::thread{[clientNode]() { rclcpp::spin(clientNode); }}.detach();                             // Spin the node in a separate thread so we can continue
  
    while (rclcpp::ok())
    {
        // Get user input
        RCLCPP_INFO(clientNode->get_logger(), "Enter a command. Type 'options' to a see a list.");
        std::string commandPrompt;
        std::getline(std::cin, commandPrompt);

        // Handle "close" command to shut down the client
        if (commandPrompt == "options")
        {
            RCLCPP_INFO(clientNode->get_logger(),
                        "Here is a list of possible commands:\n"
                        " - aft\n"
                        " - close\n"
                        " - cancel\n"
                        " - down\n"
                        " - fore\n"
                        " - home\n"
                        " - left\n"
                        " - random\n"
                        " - right\n"
                        " - up");
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
        else if (commandPrompt == "home" or commandPrompt == "random")
        {
            if (activeClient != nullptr
            and activeClient->is_running())
            {
                stop_robot(activeClient);
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
            
            activeClient = &jointTrajectoryClient;
        }
        // Handle cartesian trajectory commands
        else if (commandPrompt == "left" or commandPrompt == "right" or
                 commandPrompt == "up"   or commandPrompt == "down"  or
                 commandPrompt == "fore" or commandPrompt == "aft"   or
                 commandPrompt == "hold")
        {
            stop_robot(activeClient);
            
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
        
            activeClient = &cartesianTrajectoryClient;
        }
        else
        {
            RCLCPP_WARN(clientNode->get_logger(), "Unknown command. Type 'options' to see a list.");
        }
    }

    rclcpp::shutdown();                                                                             // Shut down

    return 0;
}
