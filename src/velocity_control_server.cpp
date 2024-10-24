/**
 * @file   velocity_control_server.cpp
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  This is testing & demonstration.
 */

#include <RobotLibrary/SerialKinematicControl.h>                                                    // For serial link robots
#include <ModelUpdater.h>                                                                           // Joint state subscriber
#include <TrackCartesianTrajectory.h>
#include <TrackJointTrajectory.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);  // Launches ROS2

    try
    {
        auto paramNode = std::make_shared<rclcpp::Node>("param_loader");                            // Temporary node to load parameters

        // Load all parameters at once
        std::string urdfLocation = paramNode->declare_parameter<std::string>("urdf_location", "");
        double kp = paramNode->declare_parameter<double>("kp", 10.0);
        double frequency = paramNode->declare_parameter<double>("frequency", 500.0);
        std::string endpointName = paramNode->declare_parameter<std::string>("endpoint_name", "unnamed");
        std::string controlTopicName = paramNode->declare_parameter<std::string>("control_topic_name", "joint_commands");

        paramNode.reset();                                                                          // Free the node and its resources

        // Create model & controller
        KinematicTree robotModel(urdfLocation);                                                     // Create the robot model
        SerialKinematicControl controller(&robotModel, endpointName, frequency);                    // Create controller, attach model
        controller.set_joint_gains(kp, 1.0);                                                        // Set control gains
        
        // Create nodes
        auto modelUpdaterNode = std::make_shared<ModelUpdater>(&robotModel);                        // Reads & updates joint state
        auto serverNode = std::make_shared<rclcpp::Node>(robotModel.name() + "_action_server");     // Create server node

        // List actions, attach server node
        TrackJointTrajectory jointTrajectoryServer(serverNode,
                                                   &controller,
                                                   "track_joint_trajectory",
                                                   "joint_command_relay");
                                                   
        TrackCartesianTrajectory cartesianTrajectoryServer(serverNode,
                                                           &controller,
                                                           "track_cartesian_trajectory",
                                                           "joint_command_relay");

        // Add nodes to executor & spin
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(modelUpdaterNode);                                                        // Add model updater node
        executor.add_node(serverNode);                                                              // Add server node

        executor.spin();                                                                            // Run the node(s)

        rclcpp::shutdown();                                                                         // As it says
        
        return 0;                                                                                   // No errors
    }
    catch (const std::exception &exception)
    {
        std::cerr << exception.what() << "\n";
        
        return -1;                                                                                  // Error occurred
    }
}

