#include <random>                                                                                   // For generating random numbers
#include "rclcpp/rclcpp.hpp"                                                                        // ROS2 C++ library
#include "rclcpp_action/rclcpp_action.hpp"                                                          // ROS2 C++ action library
#include <thread>                                                                                   // Threading (duh!)
#include "serial_link_action_server/action/track_joint_trajectory.hpp"                              // Custom action
#include "serial_link_action_server/action/track_cartesian_trajectory.hpp"
#include "serial_link_action_server/msg/cartesian_trajectory_point.hpp"

using TrackJointTrajectory = serial_link_action_server::action::TrackJointTrajectory;
using TrackCartesianTrajectory = serial_link_action_server::action::TrackCartesianTrajectory;

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
    // Ensure exactly 2 arguments are provided (executable and number of joints)
    if (argc != 2)
    {
        std::cerr << "[ERROR] [DEMO CLIENT] Usage:\n"
                  << "ros2 run serial_link_action_server demo_client numberOfJoints\n";
        return 1;                                                                                   // Exit with error
    }

    // Initialize ROS2
    rclcpp::init(argc, argv);
    int numJoints = std::stoi(argv[1]);

    auto clientNode = rclcpp::Node::make_shared("serial_link_demo_client");                         // Create a ROS2 node

    // Create action clients for joint and cartesian trajectories
    auto jointTrajectoryClient = rclcpp_action::create_client<TrackJointTrajectory>(
        clientNode, "track_joint_trajectory");
    auto cartesianTrajectoryClient = rclcpp_action::create_client<TrackCartesianTrajectory>(
        clientNode, "track_cartesian_trajectory");

    // Wait for servers to be advertised
    RCLCPP_INFO(clientNode->get_logger(), "Waiting for action servers to be advertised...");
    if (!jointTrajectoryClient->wait_for_action_server(std::chrono::seconds(5)) &&
        !cartesianTrajectoryClient->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(clientNode->get_logger(),
                     "Action servers did not appear after 5 seconds. Shutting down.");
        rclcpp::shutdown();
        return 1;                                                                                   // Exit with error
    }

    bool clientActive = true;  // Main loop control flag
    std::shared_ptr<std::thread> actionThread = nullptr;                                            // Thread for asynchronous execution

    while (clientActive && rclcpp::ok())
    {
        RCLCPP_INFO(clientNode->get_logger(),
                    "Enter command (close, home, random, up, down, left, right, fore, aft):");

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
        else if (commandPrompt == "home" or commandPrompt == "random")
        {
            serial_link_action_server::msg::JointTrajectoryPoint endPoint;
            endPoint.time = 5.0;  // Set movement time

            // Populate joint positions
            if (commandPrompt == "home")
            {
                RCLCPP_INFO(clientNode->get_logger(), "Moving to home position.");
                endPoint.position.assign(numJoints, 0.0);                                           // All joints to 0
                endPoint.velocity.assign(numJoints, 0.0);
                endPoint.acceleration.assign(numJoints, 0.0);
            }
            else if (commandPrompt == "random")
            {
                RCLCPP_INFO(clientNode->get_logger(), "Moving to random joint position.");
                std::uniform_real_distribution<double> uniformDistribution(-2.0, 2.0);
                std::default_random_engine randomEngine(static_cast<unsigned>(std::time(0)));

                // Assign random positions
                for (int i = 0; i < numJoints; ++i)
                {
                    endPoint.position.push_back(uniformDistribution(randomEngine));
                    endPoint.velocity.push_back(0.0);
                    endPoint.acceleration.push_back(0.0);
                }
            }

            // Create goal and attach trajectory points
            auto goal = std::make_shared<TrackJointTrajectory::Goal>();
            goal->points = {endPoint};
            goal->delay = 0.0;

            // Join any previous thread to avoid conflicts
            if (actionThread && actionThread->joinable())
            {
                actionThread->join();
            }

            // Start a new action thread
            actionThread = std::make_shared<std::thread>(
                [jointTrajectoryClient, goal, clientNode]()
                {
                    execute_action<TrackJointTrajectory>(jointTrajectoryClient, goal, clientNode);
                });
        }
        // Handle cartesian trajectory commands
        else if (commandPrompt == "left" or commandPrompt == "right" or
                 commandPrompt == "up"   or commandPrompt == "down"  or
                 commandPrompt == "fore" or commandPrompt == "aft")
        {
            RCLCPP_INFO(clientNode->get_logger(), "Moving endpoint %s.", commandPrompt.c_str());

            // Set up cartesian trajectory point
            serial_link_action_server::msg::CartesianTrajectoryPoint endPoint;
            endPoint.time = 2.0;
            endPoint.reference = 2;  // Relative to endpoint frame

            // Update endpoint position based on the command
            if (commandPrompt == "left")       endPoint.pose.position.y =  0.2;
            else if (commandPrompt == "right") endPoint.pose.position.y = -0.2;
            else if (commandPrompt == "up")    endPoint.pose.position.z =  0.2;
            else if (commandPrompt == "down")  endPoint.pose.position.z = -0.2;
            else if (commandPrompt == "fore")  endPoint.pose.position.x =  0.2;
            else if (commandPrompt == "aft")   endPoint.pose.position.x = -0.2;

            endPoint.pose.orientation.w = 1.0;  // No orientation change

            // Create goal and attach cartesian point
            auto goal = std::make_shared<TrackCartesianTrajectory::Goal>();
            goal->points = {endPoint};
            goal->delay = 0.0;

            // Join any previous thread to avoid conflicts
            if (actionThread && actionThread->joinable())
            {
                actionThread->join();
            }

            // Start a new action thread
            actionThread = std::make_shared<std::thread>(
                [cartesianTrajectoryClient, goal, clientNode]()
                {
                    execute_action<TrackCartesianTrajectory>(cartesianTrajectoryClient, goal, clientNode);
                });
        }
        else
        {
            RCLCPP_INFO(clientNode->get_logger(), "Unknown command. Please try again.");
        }
    }

    // Cleanup before exiting
    if (actionThread && actionThread->joinable())
    {
        actionThread->join();                                                                       // Ensure the last thread finishes before shutdown
    }

    rclcpp::shutdown();                                                                             // Shutdown ROS2
    return 0;
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

