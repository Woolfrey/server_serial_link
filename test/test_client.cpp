/**
 * @file   test_client.cpp
 * @author Jon Woolfrey
 * @date   July 2024
 * @brief  This is for testing action servers.
 */

#include <ActionServer/TrackJointTrajectory.h>
#include <random>                                                                                   // For generating random numbers

using JointControlAction = serial_link_interfaces::action::TrackJointTrajectory;


int main(int argc, char **argv)
{
    if(argc != 3)
    {
        std::cerr << "[ERROR] [TEST CLIENT] Incorrect number of arguments. Usage:\n"
                  << "ros2 run serial_link_action_server test_client robot_name numberOfJoints\n";
        
        return -1;
    }
    
    std::string actionName = "track_joint_trajectory";                                              // Name of action
    
    int numJoints = std::stoi(argv[2]);                                                             // Convert to int
    
    rclcpp::init(argc, argv);                                                                       // Launches ROS2
    
    //////////////////////////////////////// Create client /////////////////////////////////////////
    std::string name = std::string(argv[1]) + "_test_client";
    
    auto node = rclcpp::Node::make_shared(name);                                                    // Creates a node
    
    auto clientServer = rclcpp_action::create_client<JointControlAction>(node, actionName);         // Attach the node to this client
    
    RCLCPP_INFO(node->get_logger(),
                std::string("Waiting for action '" + actionName + "' to be advertised...").c_str());
   
    ////////////////////////////////////// Wait for server /////////////////////////////////////////
    if(not clientServer->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(node->get_logger(),
                     std::string("Action server '" + actionName + "' did not appear after waiting for 5 seconds. "
                                 "Shutting down.").c_str());
        
        return -1;
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Action server found.");
    }
    
    //////////////////////////////////// Create trajectory points //////////////////////////////////
    auto goal = JointControlAction::Goal();
    
    std::uniform_real_distribution<double> uniformDistribution(-1.0, 1.0);
    std::default_random_engine randomEngine;
    
    serial_link_interfaces::msg::JointTrajectoryPoint startPoint, endPoint;
    
    startPoint.time =  0.0;
    endPoint.time   = 10.0;
    
    for(int i = 0; i < numJoints; i++)
    {
         startPoint.position.push_back(uniformDistribution(randomEngine));
         startPoint.velocity.push_back(0.0);
         startPoint.acceleration.push_back(0.0);
         
         endPoint.position.push_back(uniformDistribution(randomEngine));
         endPoint.velocity.push_back(0.0);
         endPoint.acceleration.push_back(0.0);
    }
    
    goal.points = {startPoint, endPoint};                                                           // Add them to the goal
    goal.delay = 0.0;
   
    /////////////////////////////////////// Send to server /////////////////////////////////////////
    RCLCPP_INFO(node->get_logger(), "Sending goal.");
    auto sendGoalFuture = clientServer->async_send_goal(goal);
    if(rclcpp::spin_until_future_complete(node, sendGoalFuture)!= rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to send goal.");
        return -1;
    }
    
    auto goalHandle = sendGoalFuture.get();
    if(not goalHandle)
    {
        RCLCPP_ERROR(node->get_logger(), "Goal was rejected.");
        return -1;
    }
    
    ///////////////////////////////////// Wait for result //////////////////////////////////////////
    auto resultFuture = clientServer->async_get_result(goalHandle);
    if(rclcpp::spin_until_future_complete(node, resultFuture) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get result.");
        return -1;
    }
    
    auto result = resultFuture.get();
    if(result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(node->get_logger(), "Action succeeded.");
        
        for(auto &error : result.result->position_error)
        {
            std::cout << "---\n"
                      << "Mean: " << error.mean << "\n"
                      << "Variance: " << error.variance << "\n"
                      << "Min: " << error.min << "\n"
                      << "Max: " << error.max << "\n";
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Action failed.");
    }

    rclcpp::shutdown();                                                                             // Shuts down ROS2   
    
    return 0;                                                                                       // Shut down program; no errors.
}
