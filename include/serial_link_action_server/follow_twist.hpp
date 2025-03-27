
/**
 * @file    follow_twist.hpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   A ROS2 action that enables a robot arm to follow a twist command.
 * 
 * @details This class subscribes to a geometry_msgs::msg::Twist which is then executed by the robot
 *          using resolved motion rate control.
 *
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the control class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */
 
#ifndef FOLLOW_TWIST_H
#define FOLLOW_TWIST_H

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <RobotLibrary/Trajectory/CartesianSpline.h>                                                // Trajectory generator
#include <serial_link_action_server/action_server_base.hpp>                                         // Base class
#include <serial_link_interfaces/action/follow_twist.hpp>                                           // Custom generated action
#include <Utilities.h>

namespace serial_link_action_server {

/**
 * @brief This class performs joint trajectory tracking for a serial link robot arm.
 */
class FollowTwist : public serial_link_action_server::ActionServerBase<serial_link_interfaces::action::FollowTwist>
{
    public:

        // For brevity:
        using Action  = serial_link_interfaces::action::FollowTwist;
        using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
                
        /**
         * @brief Constructor for the class.
         * @param node A (shared) pointer to a ROS2 node
         * @param controller A shared pointer to a RobotLibrary controller
         * @param mutex A shared pointer to a mutex for blocking other actions
         * @param actionName What this action will be listed as on the ROS2 network
         * @param controlTopicName The name to which joint control outputs are published.
         * @param twistTopicName The name to subscribe to for input commands.
         */
        FollowTwist(std::shared_ptr<rclcpp::Node> node,
                    std::shared_ptr<RobotLibrary::Control::SerialLinkBase> controller,
                    std::shared_ptr<std::mutex> mutex,
                    const std::string &actionName = "track_cartesian_trajectory",
                    const std::string &controlTopicName = "joint_commands",
                    const std::string &twistTopicName = "twist_command");
    
    private:

        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr _twistSubscriber;         ///< Subscribes to twist topic
        
        serial_link_interfaces::msg::Statistics _angularError;                                      ///< Statistical summary of orientation tracking performance
        
        serial_link_interfaces::msg::Statistics _linearError;                                       ///< Statistical summary of position tracking performance
        
        std::optional<geometry_msgs::msg::TwistStamped> _lastTwist;                                 ///< Save the last received message from the publisher
        
        std::mutex _twistMutex;                                                                     ///< Prevents subscriber & controller accessing twist simultaneously
 
        /**
         * @brief Processes action requests from the server.
         * @param uuid A unique identification for this goal request.
         * @param goal The goal message of the FollowTwist.action definition.
         * @return Rejects if the goal is invalid, or twist action not available.
         */
        rclcpp_action::GoalResponse
        handle_goal(const rclcpp_action::GoalUUID &uuid,
                    std::shared_ptr<const Action::Goal> goal); 
        /**
         * @brief The main control loop for following twist commands.
         * @param goalHandle A shared pointer to the goal hande object.
         */
        void
        execute(const std::shared_ptr<GoalHandle> goalHandle);
        
        
        /**
         * @brief Updates the twist command when new messages are received.
         * @param
         */
        void
        twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
        
        /**
         * @brief Completes the action and sends result to client.
         * @param status 1 = Success, 2 = cancel, 3 = abort
         * @param message Relevant information for the client.
         * @param goalHandle This specific action goal handle object.
         */
        void
        cleanup_and_send_result(const int &status,
                                const std::string &message,
                                const std::shared_ptr<GoalHandle> goalHandle);
};                                                                                                  // Semicolon required after a class declaration

}

#endif
