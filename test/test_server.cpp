/**
 * @file   test_server.cpp
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  This is for testing action servers.
 */

#include <ActionServer/TrackJointTrajectory.h>
#include <RobotLibrary/Model/KinematicTree.h>
#include <RobotLibrary/Control/SerialKinematicControl.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                                                                       // Launches ROS2
    
    KinematicTree<double> model("path.urdf"); // I NEED TO FIX THIS
    
    SerialKinematicControl<double> controller(&model, "frame_name");
    
    auto action_server = std::make_shared<TrackJointTrajectory>();                                  // Create the server

    rclcpp::spin(action_server);                                                                    // Run the server

    rclcpp::shutdown();                                                                             // Shuts down ROS2   
    
    return 0;                                                                                       // Shut down program; no errors.
}
