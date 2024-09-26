/**
 * @file   velocity_control_server.cpp
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  This is testing & demonstration.
 */

#include <RobotLibrary/SerialDynamicControl.h>                                                      // For serial link robots
#include <ModelUpdater.h>                                                                           // Joint state subscriber
#include <TrackCartesianTrajectory.h>
#include <TrackJointTrajectory.h>

using TrackJointTrajectoryAction = serial_link_action_server::action::TrackJointTrajectory;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                                                                       // Launches ROS2
    
    // Load parameters
    auto paramNode = std::make_shared<rclcpp::Node>("control_parameters");

    double kp = paramNode->declare_parameter<double>("kp", 500.0);
    double kd = paramNode->declare_parameter<double>("kds", 50.0);
    std::string urdfLocation = paramNode->declare_parameter<std::string>("urdf_location", "");
    std::string endpointName = paramNode->declare_parameter<std::string>("endpoint_name", "unnamed");
    std::string controlTopicName = paramNode->declare_parameter<std::string>("control_topic_name", "joint_commands");
    
    try
    {
        KinematicTree robotModel(urdfLocation);                                                     // Create model
        
        SerialDynamicControl controller(&robotModel, endpointName);                                 // Create controller, attach model
        
        controller.set_joint_gains(kp,kd);                                                          // Gains for joint feedback
        
        std::mutex mutex;                                                                           // Blocks 2 actions running simultaneously
        
        // Create the nodes necessary for coordinating the control server
        auto jointTrajectoryServer = std::make_shared<TrackJointTrajectory>(&controller, &mutex, controlTopicName); // Runs joint control mode
        auto cartesianTrajectoryServer = std::make_shared<TrackCartesianTrajectory>(&controller, &mutex, controlTopicName); // Runs Cartesian control mode
        auto modelUpdater = std::make_shared<ModelUpdater>(&robotModel);                            // Reads & updates joint state
        
        // Create multi-thread executor and add nodes
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(modelUpdater);
        executor.add_node(jointTrajectoryServer);
        executor.add_node(cartesianTrajectoryServer);

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
