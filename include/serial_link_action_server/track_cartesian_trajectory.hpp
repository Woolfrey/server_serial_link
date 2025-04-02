/**
 * @file    track_cartesian_trajectory.hpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   A ROS2 action that enables the endpoint of a robot arm to follow a trajectory.
 * 
 * @details This class creates & advertises a ROS2 for Cartesian trajectory tracking. Given a set of
 *          poses and times, it generates a spline trajectory. It then performs real-time feedback
 *          control to make the endpoint of the robot arm follow this trajectory.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the control class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */
#ifndef TRACK_CARTESIAN_TRAJECTORY_H
#define TRACK_CARTESIAN_TRAJECTORY_H

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <RobotLibrary/Trajectory/CartesianSpline.h>                                                // Trajectory generator
#include <serial_link_action_server/action_server_base.hpp>
#include <serial_link_action_server/utilities.hpp>
#include <serial_link_interfaces/action/track_cartesian_trajectory.hpp>                             // Custom generated action

namespace serial_link_action_server {

/**
 * @brief This class performs joint trajectory tracking for a serial link robot arm.
 */
class TrackCartesianTrajectory : public ActionServerBase<serial_link_interfaces::action::TrackCartesianTrajectory>
{
    public:

        using Action  = serial_link_interfaces::action::TrackCartesianTrajectory;                // For brevity       
        
        using ActionManager = rclcpp_action::ServerGoalHandle<Action>;                              // For brevity
                
        /**
         * @brief Constructor for the class.
         * @param node A (shared) pointer to a ROS2 node
         * @param controller A shared pointer to a RobotLibrary controller
         * @param mutex A shared pointer to a mutex for blocking other actions
         * @param actionName What this action will be listed as on the ROS2 network
         * @param controlTopicName For the publisher
         */
        TrackCartesianTrajectory(std::shared_ptr<rclcpp::Node> node,
                                 std::shared_ptr<RobotLibrary::Control::SerialLinkBase> controller,
                                 std::shared_ptr<std::mutex> mutex,
                                 const std::string &actionName = "track_cartesian_trajectory",
                                 const std::string &controlTopicName = "joint_commands");
    
    private:

        serial_link_interfaces::msg::Statistics _orientationError;                                  ///< Statistical summary of orientation tracking performance
        
        serial_link_interfaces::msg::Statistics _positionError;                                     ///< Statistical summary of position tracking performance
       
        RobotLibrary::Trajectory::CartesianSpline _trajectory;                                      ///< Trajectory generator

        /**
         * @brief Processes the request to execute action.
         * @param uuid I have no idea what this does ¯\_(ツ)_/¯
         * @param request The goal component of the action definition.
         * @return REJECT if the arguments are not sound, ACCEPT_AND_EXECUTE otherwise.
         */
        rclcpp_action::GoalResponse
        handle_goal(const rclcpp_action::GoalUUID &uuid,
                    std::shared_ptr<const typename Action::Goal> request);
        
        /**
         * @brief This is the main control loop for joint trajectory tracking.
         * @param actionManager A pointer to the rclcpp::ServerGoalHandle for this action
         */
        void
        execute(const std::shared_ptr<ActionManager> actionManager);    
        
        /**
         * @brief Completes the action and send result to the client.
         * @param status 1 = Success, 2 = Cancelled, 3 = Aborted
         * @param message Information for the client.
         */
        void
        cleanup_and_send_result(const int &status,
                                const std::string &message,
                                const std::shared_ptr<ActionManager> actionManager);
        
};                                                                                                  // Semicolon required after a class declaration

}

#endif
