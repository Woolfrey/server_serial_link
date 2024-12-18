/**
 * @file   JointTrajectoryClient.h
 * @author Jon Woolfrey
 * @data   November 2024
 * @brief  A ROS2 action client for the TrackJointTrajectory action.
 */

#ifndef JOINT_TRAJECTORY_CLIENT_H
#define JOINT_TRAJECTORY_CLIENT_H

#include "ActionClientBase.h"
#include "serial_link_action_server/action/track_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

class JointTrajectoryClient : public ActionClientBase<serial_link_action_server::action::TrackJointTrajectory>
{
    public:
    
        using Action = serial_link_action_server::action::TrackJointTrajectory;                     // For brevity
    
        /**
         * Constructor.
         * @param clientNode A pointer to the client node for this server.
         * @param actionName The name of the action being advertised by the server.
         */
        JointTrajectoryClient(std::shared_ptr<rclcpp::Node> clientNode,
                              const std::string &actionName,
                              bool verbose = false)
                            : ActionClientBase(clientNode, actionName),
                              _verbose(verbose)
        {
            // Override the result callback in the base class
            _options.result_callback = std::bind
            (
                &JointTrajectoryClient::result_callback,                                            // Name of the method
                this,                                                                               // Attach this node
                std::placeholders::_1                                                               // I don't know what this does
            );
        }
                            
    private:
        
        bool _verbose = false;                                                                      ///< Used to control detail of results     
        
        /**
         * This method executes after an action is completed.
         * It overrides the method defined in the base class.
         * @param result The result portion of the action.
         */
        void
        result_callback(const rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result);
};

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Processes the result of an action                               //                           
////////////////////////////////////////////////////////////////////////////////////////////////////
void
JointTrajectoryClient::result_callback(const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result)
{
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            if(_verbose)
            {
                std::string performanceResults = "";
                
                int jointNum = 1;
                
                for(auto stats : result.result->position_error)
                {
                    performanceResults += "Joint " + std::to_string(jointNum) + ":\n"
                                          "   - Mean:      " + std::to_string(stats.mean) + "\n"
                                          "   - Std. dev.: " + std::to_string(sqrt(stats.variance)) + "\n"
                                          "   - Min.:      " + std::to_string(stats.min) + "\n"
                                          "   - Max.:      " + std::to_string(stats.max) + "\n";
                    
                    ++jointNum;
                }
                
                RCLCPP_INFO(_node->get_logger(),
                            "Joint trajectory tracking complete. Position error:\n%s",
                            performanceResults.c_str());
            }
            else
            {
                RCLCPP_INFO(_node->get_logger(), "Joint trajectory tracking complete.");
            }
            
            break;
        }
        case rclcpp_action::ResultCode::CANCELED:
        {
            RCLCPP_INFO(_node->get_logger(), "Joint trajectory tracking was canceled.");
            break;
        }
        case rclcpp_action::ResultCode::ABORTED:
        {
            RCLCPP_INFO(_node->get_logger(), "Joint trajectory tracking was aborted.");
            break;
        }
        default:
        {
            RCLCPP_WARN(_node->get_logger(), "Unknown result code.");
            break;
        }
    }
}

#endif
