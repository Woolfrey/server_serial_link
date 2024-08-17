#include <random>                                                                                   // For generating random numbers
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <thread>                                                                                   // Threading (duh!)
#include "serial_link_action_server/action/track_joint_trajectory.hpp"

using TrackJointTrajectory = serial_link_action_server::action::TrackJointTrajectory;

/**
 * This function manages goal execution in its own thread so the client can manage other things.
 * @param client A client related to a specific action.
 * @param goal The goal related to the specific action.
 * @param node The node to which said client is attached.
 * @return False if there are problems.
 */
template <typename Action>
bool
execute_action(std::shared_ptr<rclcpp_action::Client<Action>> client,
               typename Action::Goal::SharedPtr goal,
               std::shared_ptr<rclcpp::Node> node);

  /////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          MAIN                                                   //
/////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    if(argc != 2)
    {
        std::cerr << "[ERROR] [DEMO CLIENT] Use:\n"
                  << "ros2 run serial_link_action_server demo_client numberOfJoints\n";
        
        return 0;
    }
    
    rclcpp::init(argc, argv);                                                                       // Start up ROS2
    
    int numJoints = std::stoi(argv[1]);
    
    // Create the client
    auto clientNode = rclcpp::Node::make_shared("serial_link_demo_client");                         // Used to regulate actions
    auto jointTrajectoryClient = rclcpp_action::create_client<TrackJointTrajectory>(
        clientNode, "track_joint_trajectory");
        
    // Wait for server(s) to be advertised
    RCLCPP_INFO(clientNode->get_logger(), "Waiting for action(s) to be advertised...");
    
    if(not jointTrajectoryClient->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(clientNode->get_logger(),
                    "Action server(s) did not appear after waiting for 5 seconds. Shutting down.");
        
        rclcpp::shutdown();
        
        return 0;
    }
    
    // Run loop
    bool clientActive = true;                                                                       // Switch for the loop below                                                                                 
    std::shared_ptr<std::thread> actionThread = nullptr;                                            // Pointer to manage the thread
    
    do
    {
        RCLCPP_INFO(clientNode->get_logger(), "Enter command (close, home, random):");              // Inform user

        std::string commandPrompt;
        std::getline(std::cin, commandPrompt);                                                      // Get input from user

        if (commandPrompt == "close")
        {
            RCLCPP_INFO(clientNode->get_logger(), "Shutting down.");
            clientActive = false;                                                                   // This will break the loop
        }
        else if (commandPrompt == "home" or commandPrompt == "random")
        {
            // Set the trajectory points
            serial_link_action_server::msg::JointTrajectoryPoint endPoint;
            endPoint.time = 5.0;

            if (commandPrompt == "home")
            {
                RCLCPP_INFO(clientNode->get_logger(), "Moving to home position.");

                for (int i = 0; i < numJoints; i++)
                {
                    endPoint.position.push_back(0.0);
                    endPoint.velocity.push_back(0.0);
                    endPoint.acceleration.push_back(0.0);
                }
            }
            else if (commandPrompt == "random")
            {
                RCLCPP_INFO(clientNode->get_logger(), "Moving to random position.");

                std::uniform_real_distribution<double> uniformDistribution(-2.0, 2.0);
                std::default_random_engine randomEngine(static_cast<unsigned>(std::time(0)));

                for (int i = 0; i < numJoints; i++)
                {
                    endPoint.position.push_back(uniformDistribution(randomEngine));
                    endPoint.velocity.push_back(0.0);
                    endPoint.acceleration.push_back(0.0);
                }
            }

            // Concatenate trajectory points and attach to goal message
            auto goal = std::make_shared<TrackJointTrajectory::Goal>();
            goal->points = {endPoint};
            goal->delay = 0.0;

            if (actionThread && actionThread->joinable())
            {
                actionThread->join();                                                               // Wait for the previous thread to finish
            }

            // Start a new thread
            actionThread = std::make_shared<std::thread>(
                [jointTrajectoryClient, goal, clientNode]()
                {
                    execute_action<TrackJointTrajectory>(jointTrajectoryClient, goal, clientNode);
                });
        }
        else
        {
            RCLCPP_INFO(clientNode->get_logger(), "Unknown command. Please try again.");
        }

    } while (clientActive and rclcpp::ok());


    // Cleanup
    if (actionThread and actionThread->joinable())
    {
        actionThread->join();                                                                       // Wait for the last thread to finish
    }

    rclcpp::shutdown();                                                                             // Shut down ROS2
    
    return 0;                                                                                       // Exit with no errors
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Execute an action & handle its result                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename Action>
bool execute_action(std::shared_ptr<rclcpp_action::Client<Action>> client,
                    typename Action::Goal::SharedPtr goal,
                    std::shared_ptr<rclcpp::Node> node)
{
    RCLCPP_INFO(node->get_logger(), "Sending goal.");                                               // Inform user

    // Send the goal to the server
    auto sendGoalFuture = client->async_send_goal(*goal);
    if(rclcpp::spin_until_future_complete(node, sendGoalFuture) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to send goal.");
        return false;
    }

    auto goalHandle = sendGoalFuture.get();
    if(not goalHandle)
    {
        RCLCPP_ERROR(node->get_logger(), "Goal was rejected.");
        return false;
    }
    
    // Wait for the result
    auto resultFuture = client->async_get_result(goalHandle);
    if(rclcpp::spin_until_future_complete(node, resultFuture) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get result.");
        
        return false;
    }
    
    auto result = resultFuture.get();
    if(result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(node->get_logger(), "Action failed.");
        
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "Action completed.");
    
    return true;
}

