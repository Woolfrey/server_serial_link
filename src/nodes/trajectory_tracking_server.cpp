/**
 * @file    trajectory_tracking_server.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    June 2025
 * @version 1.1
 * @brief   Demonstrates the use of joint & cartesian trajectory tracking actions.
 * 
 * @details This executable creates two actions, TrackJointTrajectory & TrackCartesianTrajectory,
 *          and advertises them on the ROS2 network. It builds on the RobotLibrary::Control::SerialLinkBase
 *          classes to implement real-time feedback control.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the SerialLinkBase class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include <RobotLibrary/Control/SerialDynamicControl.h>
#include <RobotLibrary/Control/SerialKinematicControl.h>                                            // For serial link robots
#include <serial_link_action_server/model_updater.hpp>                                              // Joint state subscriber
#include <serial_link_action_server/hold_configuration.hpp>
#include <serial_link_action_server/hold_pose.hpp>
#include <serial_link_action_server/track_cartesian_trajectory.hpp>
#include <serial_link_action_server/track_joint_trajectory.hpp>
#include <serial_link_action_server/utilities.hpp>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            MAIN                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    using namespace serial_link_action_server;
    
    rclcpp::init(argc, argv);                                                                       // Launches ROS2

    // Ensure sufficient number of arguments are provided
    if (argc < 6)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Invalid number of arguments. "
                     "Usage: <executable> <urdf_path> <endpoint_name> <control_topic_name> <joint_state_topic_name> <control_mode>");

        rclcpp::shutdown();
        
        return 1;
    }
 
    // For clarity:
    std::string urdfPath        = argv[1];
    std::string endpointName    = argv[2];
    std::string controlTopic    = argv[3];
    std::string jointStateTopic = argv[4];
    std::string controlMode     = argv[5];
 
    try 
    {
        auto model = std::make_shared<RobotLibrary::Model::KinematicTree>(urdfPath);                // Generate dynamic model
        
        auto modelUpdaterNode = std::make_shared<ModelUpdater>(model, jointStateTopic, endpointName); // Create node for updating joint state
        
        auto serverNode = std::make_shared<rclcpp::Node>(model->name()+"_action_server");           // Create action server nodes

        std::shared_ptr<RobotLibrary::Control::SerialLinkBase> controller;                          // This allows for polymorphism
        
             if (controlMode == "VELOCITY") controller = std::make_unique<RobotLibrary::Control::SerialKinematicControl>(model, endpointName, load_control_parameters(serverNode));
        else if (controlMode == "TORQUE")   controller = std::make_unique<RobotLibrary::Control::SerialDynamicControl>(model, endpointName, load_control_parameters(serverNode));
        else
        {
            std::cerr << "[ERROR] [TRAJECTORY TRACKING SERVER] "
                      << "Invalid argument for control mode. Options are VELOCITY or TORQUE, "
                      << "but received " << controlMode << ".\n";
                               
            return -1;
        }
              
        // Declare action servers
        auto mutex = std::make_shared<std::mutex>();                                                // This stops 2 actions using the robot at the same time

        HoldConfiguration holdConfigurationServer(
            serverNode,
            controller,
            mutex,
            "hold_configuration",
            controlTopic);
        
        HoldPose holdPoseServer(
            serverNode,
            controller,
            mutex,
            "hold_pose",
            controlTopic);        
            
        TrackJointTrajectory jointTrajectoryServer(
            serverNode,
            controller,
            mutex,
            "track_joint_trajectory",
            controlTopic);
                                               
        TrackCartesianTrajectory cartesianTrajectoryServer(
            serverNode,
            controller,
            mutex,
            "track_cartesian_trajectory",
            controlTopic);
                 
        // Add nodes to executor and spin
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(modelUpdaterNode);
        executor.add_node(serverNode);
        
        executor.spin();
        
        rclcpp::shutdown();
        
        return 0;

    }
    catch(const std::exception &exception)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), exception.what());
        
        rclcpp::shutdown();
        
        return 1;
    }  
}
