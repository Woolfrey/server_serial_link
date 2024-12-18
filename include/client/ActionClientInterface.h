/**
 * @file   ActionClientInterface.h
 * @author Jon Woolfrey
 * @data   October 2024
 * @brief  This is an interface class for action clients in ROS2.
 */
 
#ifndef ACTION_CLIENT_INTERFACE_H
#define ACTION_CLIENT_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class ActionClientInterface
{
    public:
    
        /**
         * Destructor.
         */
        virtual
        ~ActionClientInterface() = default;
        
        /**
         * Gets the status of the action that is running.
         * This is a virtual method and must be defined in any derived class.
         * https://docs.ros2.org/foxy/api/action_msgs/msg/GoalStatus.html
         * 0 = Unknown
         * 1 = Accepted
         * 2 = Executing
         * 3 = Canceling
         * 4 = Succeeded
         * 5 = Canceled
         * 6 = Aborted
         * @return An int8_t for the status.
         */
        virtual
        int8_t
        status() const = 0;
        
        /**
         * This cancels an action that is currently executing.
         * This is a virtual method and must be defined in any derived class.
         * @return True if successful, false if there was a problem.
         */
        virtual
        bool
        cancel_action() = 0;
        
        /**
         * Checks to see if the action is currently running.
         * This is a virtual method and must be defined in any derived class.
         * @return True if the action is accepted, currently running, or in the process of cancelling.
         */
        virtual
        bool
        is_running() const = 0;
};                                                                                                  // Semicolon required after class declaration

#endif
