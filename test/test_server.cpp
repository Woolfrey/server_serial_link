/**
 * @file   test_server.cpp
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  This is for testing action servers.
 */

#include <ActionServer/TrackJointTrajectory.h>
#include <Subscriber/JointStateSubscriber.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                                                                       // Launches ROS2
    
    auto action_server = std::make_shared<TrackJointTrajectory>();                                  // Create the server
    
    auto joint_state_sub = std::make_shared<JointStateSubscriber>();

    rclcpp::spin(action_server);                                                                    // Run the server

    rclcpp::shutdown();                                                                             // Shuts down ROS2   
    
    return 0;                                                                                       // Shut down program; no errors.
}
