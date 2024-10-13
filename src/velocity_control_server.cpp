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
    rclcpp::init(argc, argv);                                                                       // Launches ROS2
    
    // Load parameters
    auto paramNode = std::make_shared<rclcpp::Node>("control_parameters");

    double kp = paramNode->declare_parameter<double>("kp", 10.0);
    double frequency = paramNode->declare_parameter<double>("frequency", 500.0);
    std::string urdfLocation = paramNode->declare_parameter<std::string>("urdf_location", "");
    std::string endpointName = paramNode->declare_parameter<std::string>("endpoint_name", "unnamed");
    std::string controlTopicName = paramNode->declare_parameter<std::string>("control_topic_name", "joint_commands");
 
    try
    {
        KinematicTree robotModel(urdfLocation);                                                     // Create model
        
        SerialKinematicControl controller(&robotModel, endpointName, frequency);                    // Create controller, attach model
        
        controller.set_joint_gains(kp, 1.0);                                                        // Second argument is trivial
        
        std::mutex mutex;                                                                           // Blocks 2 actions running simultaneously
 
        auto modelUpdaterNode = std::make_shared<ModelUpdater>(&robotModel);                        // Reads & updates joint state   
            
        // Create the action servers and attach them to the server node
        auto serverNode = std::make_shared<rclcpp::Node>("action_server");
        
        TrackJointTrajectory jointTrajectoryServer(serverNode,
                                                   &controller,
                                                   &mutex, 
                                                   "track_joint_trajectory",
                                                   "joint_command_relay");
                                                         
        TrackCartesianTrajectory cartesianTrajectoryServer(serverNode,
                                                           &controller,
                                                           &mutex,
                                                           "track_cartesian_trajectory",
                                                           "joint_command_relay");   
        
        // Create multi-thread executor and add nodes
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(modelUpdaterNode);
        executor.add_node(serverNode);

        executor.spin();                                                                            // Runs all the nodes on separate threads

        rclcpp::shutdown();                                                                         // Shuts down ROS2   
        
        return 0;                                                                                   // Shut down program; no errors.        
    }
    catch(const std::exception &exception)
    {
        std::cerr << exception.what() << "\n";
        
        return -1;
    }
}
