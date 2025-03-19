/**
 * @file   velocity_control_server.cpp
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  This is testing & demonstration.
 */

#include <ModelUpdater.h>                                                                           // Joint state subscriber
#include <RobotLibrary/Control/SerialKinematicControl.h>                                            // For serial link robots
#include <TrackCartesianTrajectory.h>
#include <TrackJointTrajectory.h>
#include <Utilities.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            MAIN                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                                                                       // Launches ROS2

    unsigned int returnCode = 0;

    if (argc < 5)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"),
                     "Invalid number of arguments. Usage: <executable> <urdf_path> <endpoint_name> <control_topic_name> <joint_state_topic_name>");
        returnCode = 1;
    }
    else
    {  
        // For clarity:
        std::string urdfPath        = argv[1];
        std::string endpointName    = argv[2];
        std::string controlTopic    = argv[3];
        std::string jointStateTopic = argv[4];
            
        try 
        {
            auto model = std::make_shared<RobotLibrary::Model::KinematicTree>(urdfPath);            // Generate dynamic model
            
            auto modelUpdaterNode = std::make_shared<ModelUpdater>(model, jointStateTopic);         // Create node for updating joint state
            
            auto serverNode = std::make_shared<rclcpp::Node>(model->name()+"_action_server");       // Create action server nodes
            
            auto controller = std::make_shared<RobotLibrary::Control::SerialKinematicControl>(model, endpointName, load_control_parameters(serverNode));
              
            // Declare action servers
            auto mutex = std::make_shared<std::mutex>(); 
       
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

        }
        catch(const std::exception &exception)
        {
            RCLCPP_ERROR(rclcpp::get_logger("main"), exception.what());
            
            returnCode = 1;
        }  
    }
    
    rclcpp::shutdown();

    return returnCode;
}
