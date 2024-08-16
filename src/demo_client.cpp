#include <random>                                                                                   // For generating random numbers
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <thread>                                                                                   // Threading (duh!)
#include "serial_link_action_server/action/track_joint_trajectory.hpp"

using TrackJointTrajectory = serial_link_action_server::action::TrackJointTrajectory;

// Function template remains the same
template <typename Action>
bool request_action(std::shared_ptr<rclcpp_action::Client<Action>> client,
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                                                                       // Start up ROS2
    
    // Create nodes
    auto paramNode  = rclcpp::Node::make_shared("client_parameters");                               // Used for retrieving parameters in launch file
    auto clientNode = rclcpp::Node::make_shared("serial_link_demo_client");                         // Attaches to action clients
    
    
    // Get the parameters from the launch file
    int numJoints = paramNode->declare_parameter<int>("number_of_joints", 7);
        
    
    // Create the client(s)
    auto jointControlClient = rclcpp_action::create_client<TrackJointTrajectory>(clientNode, "track_joint_trajectory");
    
    // Wait for server(s) to be advertised
    RCLCPP_INFO(clientNode->get_logger(), "Waiting for action(s) to be advertised...");
    
    if(not jointControlClient->wait_for_action_server(std::chrono::seconds(5)))
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
        // Print command prompt at the start of each iteration
        RCLCPP_INFO(clientNode->get_logger(), "Enter command (close, home, random):");

        std::string commandPrompt;
        std::getline(std::cin, commandPrompt);

        if (commandPrompt == "close")
        {
            RCLCPP_INFO(clientNode->get_logger(), "Shutting down.");
            clientActive = false; // This will break the loop
        }
        else if (commandPrompt == "home" || commandPrompt == "random")
        {
            // Prepare the goal based on the command
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

            auto goal = std::make_shared<TrackJointTrajectory::Goal>();
            goal->points = {endPoint};
            goal->delay = 0.0;

            if (actionThread && actionThread->joinable())
            {
                actionThread->join(); // Wait for the previous thread to finish
            }

            // Start a new thread
            actionThread = std::make_shared<std::thread>(
                [jointControlClient, goal, clientNode]()
                {
                    request_action<TrackJointTrajectory>(jointControlClient, goal, clientNode);
                });

            if (actionThread && actionThread->joinable())
            {
                actionThread->join(); // Wait for the current thread to finish
            }
        }
        else
        {
            RCLCPP_INFO(clientNode->get_logger(), "Unknown command. Please try again.");
        }

    } while (clientActive && rclcpp::ok());


    // Cleanup
    if (actionThread && actionThread->joinable())
    {
        actionThread->join(); // Wait for the last thread to finish
    }

    rclcpp::shutdown();
    return 0;

    
    rclcpp::shutdown(); // Shuts down ROS2   
    
    return 0; // Shut down program; no errors.
}

