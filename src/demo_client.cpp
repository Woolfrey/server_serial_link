/**
 * @file   demo_client.cpp
 * @author Jon Woolfrey
 * @date   August 2024
 * @brief  This is for testing action servers.
 */

#include "client/CartesianTrajectoryClient.h"
#include "client/JointTrajectoryClient.h"
#include <random>                                                                                   // For generating random numbers
#include "rclcpp/rclcpp.hpp"                                                                        // ROS2 C++ library
#include "rclcpp_action/rclcpp_action.hpp"                                                          // ROS2 C++ action library
#include <thread>                                                                                   // Threading (duh!)

using TrackCartesianTrajectory = serial_link_action_server::action::TrackCartesianTrajectory;
using TrackJointTrajectory     = serial_link_action_server::action::TrackJointTrajectory;
using JointTrajectoryPoint     = serial_link_action_server::msg::JointTrajectoryPoint;

/**
 * Store pre-defined joint trajectories and save them in a std::map.
 * @return A std::map where the key is the trajectory name specified in the YAML file.
 */
std::map<std::string, std::vector<JointTrajectoryPoint>>
load_joint_configurations()
{
    std::map<std::string,std::vector<JointTrajectoryPoint>> jointConfigurations;                    // We want to return this
    
    auto configNode = rclcpp::Node::make_shared("joint_configurations");                            // Create node to host parameters
    
    // Acquire number of joints
    configNode->declare_parameter("number_of_joints", 0);
    int numJoints;
    configNode->get_parameter("number_of_joints", numJoints);
    
    // Get a list of configuration names
    configNode->declare_parameter("names", std::vector<std::string>{});                             // Declare name list  
    std::vector<std::string> names = {};                                                            // Storage location for names  
    configNode->get_parameter("names", names);                                                      // Get the values from the server
    
    // Cycle through all the named configurations, extract waypoints
    for(auto name : names)
    {
        // Declare the position, time parameters
        configNode->declare_parameter(name+".positions", std::vector<double>{});
        configNode->declare_parameter(name+".times", std::vector<double>{});
        
        // Create storage location
        std::vector<double> positions;
        std::vector<double> times;
        
        // Obtain from parameter server
        configNode->get_parameter(name+".positions", positions);
        configNode->get_parameter(name+".times", times);
        
        // Ensure dimensions are correct
        if (positions.size() % numJoints != 0)
        {
            throw std::invalid_argument
            (
                "Size of position array (" + std::to_string(positions.size()) + 
                ") not divisible by number of joints (" + std::to_string(numJoints) + ")."
            );
        }
     
        // Ensure number of waypoints matches number of times
        if(positions.size()/numJoints != times.size())
        {
            throw std::invalid_argument
            (
                "Number of waypoints (" + std::to_string(positions.size()/numJoints) + ") "
                "does not match number of times (" + std::to_string(times.size()) + ")."
            );
        }
        
        // Put the positions & times together to define a trajectory
        std::vector<JointTrajectoryPoint> points;
        
        for (size_t i = 0; i < times.size(); ++i)
        {
            JointTrajectoryPoint point;
            
            point.time = times[i];
            
            std::vector<double> joint_positions(positions.begin() + i * numJoints,
                                                positions.begin() + (i + 1) * numJoints);
            
            point.position = joint_positions;                                                       // Assign the extracted positions
            
            points.push_back(point);                                                                // Add the point to the trajectory
        }
        
        jointConfigurations.emplace(name, points);
    }
    
    return jointConfigurations;
}

/**
 * This function manages the asynchronous cancellation sequence.
 * @param activeClient A pointer to the action client that is currently running.
 * @return True if/when successful, false if there was a problem.
 */
bool
stop_robot(ActionClientInterface *activeClient)
{
    if (activeClient == nullptr)
    {
        return true;                                                                                // No active action
    }

    // Possible return codes:
    // 0 = Unknown
    // 1 = Accepted
    // 2 = Executing
    // 3 = Canceling
    // 4 = Succeeded
    // 5 = Canceled
    // 6 = Aborted

    if (activeClient->status() == 0)
    {
        return false; // A problem
    }

    // If running, cancel
    if (activeClient->status() == 1
    or  activeClient->status() == 2)
    {
        activeClient->cancel_action();
    }

    // Wait until action is resolved
    while (activeClient->status() != 4 &&
           activeClient->status() != 5 &&
           activeClient->status() != 6)
    {      
        std::this_thread::sleep_for(std::chrono::milliseconds(10));                                 // Wait for 100ms
    }

    return true;
}

  /////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          MAIN                                                   //
/////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);                                                                       // Starts up ROS2
   
    auto clientNode = rclcpp::Node::make_shared("serial_link_demo_client");                         // Create client node and advertise its name

    std::map<std::string, std::vector<JointTrajectoryPoint>> jointConfigurations = load_joint_configurations();

    // Create the action clients, attach to node
    JointTrajectoryClient jointTrajectoryClient(clientNode, "track_joint_trajectory", true);
    CartesianTrajectoryClient cartesianTrajectoryClient(clientNode, "track_cartesian_trajectory", true);  
    ActionClientInterface *activeClient = nullptr;                                                  // Use this to keep track of which action is running           
   
    std::thread{[clientNode]() { rclcpp::spin(clientNode); }}.detach();                             // Spin the node in a separate thread so we can continue
  
    while (rclcpp::ok())
    {
        // Get user input
        RCLCPP_INFO(clientNode->get_logger(), "Enter a command. Type 'options' to a see a list.");
        std::string commandPrompt;
        std::getline(std::cin, commandPrompt);
        
        if (commandPrompt == "options")
        {
            RCLCPP_INFO(clientNode->get_logger(), "Available configurations:");
            for (const auto& config : jointConfigurations)
            {
                RCLCPP_INFO(clientNode->get_logger(), "- %s", config.first.c_str());
            }
        }
        else if (commandPrompt == "close")
        {
            RCLCPP_INFO(clientNode->get_logger(), "Shutting down.");
            
            stop_robot(activeClient);

            break;
        }
        else if(commandPrompt == "cancel" or  commandPrompt == "")
        {
            stop_robot(activeClient);
        }
        else
        {
            auto iterator = jointConfigurations.find(commandPrompt);
            
            if (iterator != jointConfigurations.end())
            {   
                if (activeClient != nullptr and activeClient->is_running()) stop_robot(activeClient);
                
                // Create goal and attach trajectory points
                auto goal = std::make_shared<TrackJointTrajectory::Goal>();
                
                goal->points = iterator->second;
                
                RCLCPP_INFO(clientNode->get_logger(), "Moving to `%s` configuration(s).", commandPrompt.c_str());

                jointTrajectoryClient.send_request(goal);
                
                activeClient = &jointTrajectoryClient;               
            }
            else
            {
                RCLCPP_WARN(clientNode->get_logger(), "Configuration '%s' not found.", commandPrompt.c_str());
            }
        }
        else
        {
            RCLCPP_WARN(clientNode->get_logger(), "Unknown command. Type 'options' to see a list.");
        }
    }

    rclcpp::shutdown();                                                                             // Shut down

    return 0;
}
