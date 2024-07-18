/**
 * @file   test_server.cpp
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  This is for testing action servers.
 */

#include <ActionServer/TrackJointTrajectory.h>
#include <Subscriber/ModelUpdater.h>
#include <RobotLibrary/SerialKinematicControl.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                                                                       // Launches ROS2
    
    if(argc != 3)
    {
        throw std::invalid_argument("[ERROR] [TEST SERVER] Incorrect number of arguments. "
                                    "Usage: ros2 run serial_link_action_server test_server path/to/model.urdf endpoint_name");
        
        return -1;
    }
    
    KinematicTree robotModel(argv[1]);                                                              // Create the robot
    
    SerialKinematicControl controller(&robotModel, argv[2]);
    
    // Create the nodes necessary for coordinating the control server
    auto actionServer = std::make_shared<TrackJointTrajectory>(&controller);                        // Runs joint control mode
    auto modelUpdater = std::make_shared<ModelUpdater>(&robotModel);                                // Reads & updates joint state
    
    // Create multi-thread executor and add nodes
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(actionServer);
    executor.add_node(modelUpdater);

    executor.spin();                                                                                // Runs all the nodes on separate threads

    rclcpp::shutdown();                                                                             // Shuts down ROS2   
    
    return 0;                                                                                       // Shut down program; no errors.
}
