/**
 * @file    ActionServerBase.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   Provides a common structure for all action servers.
 * 
 * @details This class elaborates on the fundamental methods required for sending goals & receiving
 *          results in a ROS2 action client. It provides common structure to all actions so they can
 *          be coordinated under a single client executable.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the KinematicTree class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#ifndef ACTION_SERVER_BASE_H
#define ACTION_SERVER_BASE_H

#include <mutex>
#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ libraries
#include <rclcpp_action/rclcpp_action.hpp>                                                          // ROS2 Action C++ libraries
#include <RobotLibrary/Control/SerialLinkBase.h>                                                    // Controller
#include <serial_link_interfaces/msg/joint_command.hpp>                                             // Custom message
#include <thread>

/**
 * @brief This is a base class to standardise actions for control of serial link robots.
 */
template <class Action>
class ActionServerBase
{ 
    public:
    
        // For brevity:
        using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
        using JointCommandMsg = serial_link_interfaces::msg::JointCommand;
            
        /**
         * @brief Constructor for the class.
         * @param node A pointer to a ROS2 node
         * @param controller A pointer to an arm controller from RobotLibrary
         * @param mutex A pointer to the mutex object
         * @param controlTopicName The name of the topic for publishing joint commands
         */
        ActionServerBase(std::shared_ptr<rclcpp::Node> node,
                         RobotLibrary::Control::SerialLinkBase *controller,
                         std::mutex *padlock,
                         const std::string &actionName = "you_forgot_to_name_me_you_rube",
                         const std::string &controlTopicName = "joint_commands");
    
    protected:

        std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> _activeGoalHandle;                 ///< Used for processing action requests
        
        std::shared_ptr<rclcpp::Node> _node;                                                        ///< Pointer to a node

        unsigned int _numJoints;                                                                    ///< Number of joints being controlled
            
        typename rclcpp_action::Server<Action>::SharedPtr _actionServer;                            ///< This is the foundation for the class.

        std::shared_ptr<typename Action::Feedback> _feedback = std::make_shared<typename Action::Feedback>(); ///< Use this to store feedback
        
        RobotLibrary::Control::SerialLinkBase* _controller;                                         ///< Pointer to the controller
        
        JointCommandMsg _jointCommand;                                                              ///< Stores information
        
        rclcpp::Publisher<JointCommandMsg>::SharedPtr _jointCommandPublisher;                       ///< Makes commands public on ROS2

        std::mutex* _padlock;                                                                       ///< Used to prevent 2 actions controlling the robot simultaneously
        
        /**
         * @brief Processes the request to execute action.
         *        This is a virtual function and must be defined by any derived class.
         * @param uuid Unique ID for the goal.
         * @param request The goal component of the action definition.
         * @return REJECT if the arguments are not sound, ACCEPT_AND_EXECUTE otherwise.
         */
        virtual
        rclcpp_action::GoalResponse
        handle_goal(const rclcpp_action::GoalUUID &uuid,
                    std::shared_ptr<const typename Action::Goal> request) = 0;

        /**
         * @brief This contains the main control loop for the action.
         *        This is a virtual function and must be defined in any derived class.
         * @param actionManager A pointer to the rclcpp::ServerGoalHandle for this action
         */
        virtual
        void
        execute(const std::shared_ptr<GoalHandle> goalHandle) = 0; 
                
        /**
         * @brief Processes the cancel request.
         *        This is a virtual function and must be defined in any derived class.
         * @param actionManager A pointer to the rclcpp::ServerGoalHandle for this action
         * @return rclcpp_action::CancelResponse::ACCEPT
         */
        rclcpp_action::CancelResponse
        handle_cancel(const std::shared_ptr<GoalHandle> goalHandle);
        
        /**
         * @brief If a goal is accepted, this method will generate a thread to run the control loop.
         *        This enables the server to process other requests without being blocked.
         * @param actionManager A pointer to the rclcpp::ServerGoalHandle
         */
        void
        handle_accepted(const std::shared_ptr<GoalHandle> goalHandle);
            
        /**
         * @brief This does exactly what it says.
         * @param command A vector of joint commands (positions, velocities, or torques)
         */
        void
        publish_joint_command(const Eigen::VectorXd &command);
        
};                                                                                                  // Semicolon required after a class declaration

#include <ActionServerBase.tpp>                                                                     // Source files

#endif
