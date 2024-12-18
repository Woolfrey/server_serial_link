/**
 * @file   CartesianTrajectoryClient.h
 * @author Jon Woolfrey
 * @data   November 2024
 * @brief  A ROS2 action client for the TrackCartesianTrajectory action.
 */

#ifndef CARTESIAN_TRAJECTORY_CLIENT_H
#define CARTESIAN_TRAJECTORY_CLIENT_H

#include "ActionClientBase.h"
#include "serial_link_action_server/action/track_cartesian_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

class CartesianTrajectoryClient : public ActionClientBase<serial_link_action_server::action::TrackCartesianTrajectory>
{
    public:
    
        using Action = serial_link_action_server::action::TrackCartesianTrajectory;                 // For brevity
    
        /**
         * Constructor.
         * @param clientNode A pointer to the client node.
         * @param actionName The name of the action being advertised by the server.
         */
        CartesianTrajectoryClient(std::shared_ptr<rclcpp::Node> clientNode,
                                  const std::string &actionName,
                                  bool verbose = false)
                                : ActionClientBase(clientNode, actionName),
                                  _verbose(verbose)
        {
            // Override the result callback in the base class
            _options.result_callback = std::bind
            (
                &CartesianTrajectoryClient::result_callback,                                        // Name of the method
                this,                                                                               // Attach this node
                std::placeholders::_1                                                               // I don't know what this does
            );
        }
                                  
    private:
       
        bool _verbose = false;                                                                      ///< Controls detail of results
        
        /**
         * This method executes after an action is completed.
         * It overrides the method defined in the base class.
         * @result The result portion of the action.
         */
        void
        result_callback(const rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result);   
};


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Processes the result of an action                               //                           
////////////////////////////////////////////////////////////////////////////////////////////////////
void
CartesianTrajectoryClient::result_callback(const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result)
{
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            if(_verbose)
            {
                std::string performanceResults =
                "Position error (mm) :\n"
                "   - Mean:      " + std::to_string(result.result->position_error.mean*1000) + "\n"
                "   - Std. dev.: " + std::to_string(sqrt(result.result->position_error.variance)*1000) + "\n"
                "   - Min:       " + std::to_string(result.result->position_error.min*1000) + "\n"
                "   - Max:       " + std::to_string(result.result->position_error.max*1000) + "\n"
                "Orientation error (deg) :\n"
                "   - Mean:      " + std::to_string(result.result->orientation_error.mean*180/M_PI) + "\n"
                "   - Std. dev.: " + std::to_string(sqrt(result.result->orientation_error.variance)*180/M_PI) + "\n"
                "   - Min:       " + std::to_string(result.result->orientation_error.min*180/M_PI) + "\n"
                "   - Max:       " + std::to_string(result.result->orientation_error.max*180/M_PI);            
     
                RCLCPP_INFO(_node->get_logger(),
                            "Joint trajectory tracking complete.\n%s",
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

