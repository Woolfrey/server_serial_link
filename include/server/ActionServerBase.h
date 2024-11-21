/**
 * @file   ActionServerBase.h
 * @author Jon Woolfrey
 * @data   October 2024
 * @brief  A base class for standardising action servers for serial link robots.
 */

#ifndef ACTIONSERVER_BASE_H
#define ACTIONSERVER_BASE_H

#include <mutex>
#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ libraries
#include <rclcpp_action/rclcpp_action.hpp>                                                          // ROS2 Action C++ libraries
#include <RobotLibrary/SerialLinkBase.h>                                                            // Controller
#include "serial_link_action_server/msg/joint_command.hpp"                                          // Custom message
#include <thread>

/**
 * This is a base class to standardise actions for control of serial link robots.
 */
template <class Action>
class ActionServerBase
{ 
    public:
    
        using ActionManager = rclcpp_action::ServerGoalHandle<Action>;                              // For brevity
        
        using JointCommandMsg = serial_link_action_server::msg::JointCommand;                       // For brevity
            
        /**
         * Constructor for the class.
         * @param node A pointer to a ROS2 node
         * @param controller A pointer to an arm controller from RobotLibrary
         * @param mutex A pointer to the mutex object
         * @param controlTopicName The name of the topic for publishing joint commands
         * @param options I have no idea what this does ¯\_(ツ)_/¯
         */
        ActionServerBase(std::shared_ptr<rclcpp::Node> node,
                         RobotLibrary::SerialLinkBase *controller,
                         std::mutex *padlock,
                         const std::string &actionName = "you_forgot_to_name_me_you_rube",
                         const std::string &controlTopicName = "joint_commands");
    
    protected:

        std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> _activeGoalHandle;                 ///< Used for processing action requests
        
        std::shared_ptr<rclcpp::Node>_node;                                                         ///< Pointer to a node

        unsigned int _numJoints;                                                                    ///< Number of joints being controlled
            
        typename rclcpp_action::Server<Action>::SharedPtr _actionServer;                            ///< This is the foundation for the class.

        std::shared_ptr<typename Action::Feedback> _feedback = std::make_shared<typename Action::Feedback>(); ///< Use this to store feedback
        
        RobotLibrary::SerialLinkBase* _controller;                                                  ///< Pointer to the controller
        
        JointCommandMsg _jointCommand;                                                              ///< Stores information
        
        rclcpp::Publisher<JointCommandMsg>::SharedPtr _jointCommandPublisher;                       ///< Makes commands public on ROS2

        std::mutex *_padlock;                                                                       ///< Used to prevent 2 actions controlling the robot simultaneously
        
        /**
         * Processes the request to execute action.
         * This is a virtual function and must be defined by any derived class.
         * @param uuid I have no idea what this does ¯\_(ツ)_/¯
         * @param request The goal component of the action definition.
         * @return REJECT if the arguments are not sound, ACCEPT_AND_EXECUTE otherwise.
         */
        virtual
        rclcpp_action::GoalResponse
        process_request(const rclcpp_action::GoalUUID &uuid,
                        std::shared_ptr<const typename Action::Goal> request) = 0;

        /**
         * This contains the main control loop for the action.
         * This is a virtual function and must be defined in any derived class.
         * @param actionManager A pointer to the rclcpp::ServerGoalHandle for this action
         */
        virtual
        void
        execute(const std::shared_ptr<ActionManager> actionManager) = 0; 
                
        /**
         * Processes the cancel request.
         * This is a virtual function and must be defined in any derived class.
         * @param actionManager A pointer to the rclcpp::ServerGoalHandle for this action
         * @return rclcpp_action::CancelResponse::ACCEPT
         */
        rclcpp_action::CancelResponse
        cancel(const std::shared_ptr<ActionManager> actionManager)
        {
            (void) actionManager;                                                                   // This stops colcon from issuing a warning
            
            RCLCPP_INFO(_node->get_logger(), "Received request to cancel action.");
            
            return rclcpp_action::CancelResponse::ACCEPT;
        }
        
        /**
         * If a goal is accepted, this method will generate a thread to run the control loop.
         * This enables the server to process other requests without being blocked.
         * @param actionManager A pointer to the rclcpp::ServerGoalHandle
         */
        void
        prepare(const std::shared_ptr<ActionManager> actionManager)
        {
            (void) actionManager;                                                                   // This stops colcon from issuing a warning
            
            std::thread{std::bind(&ActionServerBase::execute, this, std::placeholders::_1), actionManager}.detach();
        }
            
        /**
         * This does exactly what it says.
         * @param command A vector of joint commands (positions, velocities, or torques)
         */
        void
        publish_joint_command(const Eigen::VectorXd &command);
        
};                                                                                                  // Semicolon required after a class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
ActionServerBase<Action>::ActionServerBase(std::shared_ptr<rclcpp::Node> node,
                                           RobotLibrary::SerialLinkBase *controller,
                                           std::mutex *padlock,
                                           const std::string &actionName,
                                           const std::string &controlTopicName)
                                           : _node(node),
                                             _numJoints(controller->model()->number_of_joints()),
                                             _controller(controller),
                                             _padlock(padlock)
{
    using namespace std::placeholders;

    // Initialise the action server, link methods
    _actionServer = rclcpp_action::create_server<Action>(
        _node,
        actionName,
        std::bind(&ActionServerBase::process_request, this, _1, _2),
        std::bind(&ActionServerBase::cancel, this, _1),
        std::bind(&ActionServerBase::prepare,this,_1)
    );
    
    // Pre-assign joint names
    for(unsigned int i = 0; i < _numJoints-1; ++i)
    {   
        _jointCommand.name.push_back(controller->model()->joint(i).name());
    }
      
    _jointCommandPublisher = _node->create_publisher<JointCommandMsg>(controlTopicName, 1);         // Create publisher for joint commands
                                                  
    RCLCPP_INFO(_node->get_logger(),
                "'%s' action initiated. Publishing control output to '%s'.",
                actionName.c_str(),
                controlTopicName.c_str());
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                  Publish a joint command                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
void
ActionServerBase<Action>::publish_joint_command(const Eigen::VectorXd &command)
{
    _jointCommand.stamp = _node->now();                                                             // Add time of publication
    _jointCommand.command = {command.data(), command.data() + command.size()};                      // Tansfer data
    _jointCommandPublisher->publish(_jointCommand);                                                 // Publish control topic
}

#endif
