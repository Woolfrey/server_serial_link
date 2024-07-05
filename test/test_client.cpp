/**
 * @file   test_client.cpp
 * @author Jon Woolfrey
 * @date   July 2024
 * @brief  This is for testing action servers.
 */

#include <ActionServer/TrackJointTrajectory.h>

using JointControlAction = serial_link_interfaces::action::TrackJointTrajectory;

std::string actionName = "track_joint_trajectory";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                                                                       // Launches ROS2
    
    //////////////////////////////////////// Create client /////////////////////////////////////////
    auto node = rclcpp::Node::make_shared("test_node");                                             // Creates a node
    
    auto clientServer = rclcpp_action::create_client<JointControlAction>(node, actionName);         // Attach the node to this client
    
    RCLCPP_INFO(node->get_logger(),
                std::string("Waiting for action '" + actionName + "' to be advertised...").c_str());
   
    ////////////////////////////////////// Wait for server ///////////////////////////////////////////
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
    // NEED TO FILL IN DETAILS HERE
    
    serial_link_interfaces::msg::JointTrajectoryPoint startPoint;
    startPoint.time         = 0.0;
    startPoint.position     = {0.0, 1.0,-1.0};
    startPoint.velocity     = {0.0, 0.0, 0.0};
    startPoint.acceleration = {0.0, 0.0, 0.0};
    
    serial_link_interfaces::msg::JointTrajectoryPoint endPoint;
    endPoint.time         = 3.0;
    endPoint.position     = {1.0, 0.0, 0.0};
    endPoint.velocity     = {0.0, 0.0, 0.0};
    endPoint.acceleration = {0.0, 0.0, 0.0};
    
    goal.points = {startPoint, endPoint};                                                           // Add them to the goal
    goal.delay = 5.0;
   
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
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Action failed.");
    }

    rclcpp::shutdown();                                                                             // Shuts down ROS2   
    
    return 0;                                                                                       // Shut down program; no errors.
}
