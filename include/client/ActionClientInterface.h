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
         */
        virtual
        int8_t
        status() const = 0;
        
        virtual
        bool
        cancel_action() = 0;
        
        virtual
        bool
        is_running() const = 0;
};                                                                                                  // Semicolon required after class declaration

#endif
