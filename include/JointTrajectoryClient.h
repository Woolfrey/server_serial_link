#ifndef JOINT_TRAJECTORY_CLIENT_H
#define JOINT_TRAJECTORY_CLIENT_H

#include "ActionClientBase.h" // Include your base class header
#include "serial_link_action_server/action/track_joint_trajectory.hpp" // Adjust the include path as needed
#include <rclcpp/rclcpp.hpp>



class JointTrajectoryClient : public ActionClientBase<serial_link_action_server::action::TrackJointTrajectory>
{
    public:
    
        using Action = serial_link_action_server::action::TrackJointTrajectory;
    
        JointTrajectoryClient(std::shared_ptr<rclcpp::Node> clientNode,
                              const std::string &actionName)
                              : ActionClientBase(clientNode, actionName)
        {
            // Worker bees can leave.
            // Even drones can fly away.
            // The Queen is their slave.
        }  
    
    private:
        
        /**
         * Processes the response to an action request.
         * @param future A future containing the result of the goal send operation.
         */
        void process_response(const typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr &goalHandle)
        {
            if (!goalHandle)
            {
                RCLCPP_ERROR(_node->get_logger(), "Goal was rejected.");
                return;
            }

            RCLCPP_INFO(_node->get_logger(), "Response received!");
        }


        /**
         * Processes the result of an action.
         * @param result The wrapped result of the action.
         */
        void process_result(const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result)
        {
            (void) result;
            
            RCLCPP_INFO(_node->get_logger(), "Action complete.");
        }
};

#endif // JOINT_TRAJECTORY_CLIENT_H

