/**
 * @file    TrackJointTrajectory.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   A ROS2 action for tracking joint trajectories of a robot arm.
 * 
 * @details This class is used to advertise a ROS2 action server for joint trajectory tracking.
 *          Given a series of joint configurations, and times at which to reach them, it will generate
 *          a spline trajectory to connect them. Real-time feedback control is performed to track it.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the SerialLink control class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#ifndef TRACKJOINTTRAJECTORY_H
#define TRACKJOINTTRAJECTORY_H

#include <ActionServerBase.h>
#include <RobotLibrary/Trajectory/SplineTrajectory.h>
#include <serial_link_interfaces/action/track_joint_trajectory.hpp>

/**
 * @brief This class performs joint trajectory tracking for a serial link robot arm.
 */
class TrackJointTrajectory : public ActionServerBase<serial_link_interfaces::action::TrackJointTrajectory>
{
    public:
    
        using Action = serial_link_interfaces::action::TrackJointTrajectory;                        // For brevity of code
        using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;                                 // For brevity of code
                   
        /**
         * @brief Constructor for the class.
         * @param node A shared pointer to a node
         * @param controller A pointer to a robot arm controller
         * @param mutex An object that blocks 2 actions controlling the robot
         * @param actionName The name to be advertised on the ROS2 network
         * @param controlTopicName The name for the joint command publisher
         * @param options I have no idea what this does ¯\_(ツ)_/¯
         */
        TrackJointTrajectory(std::shared_ptr<rclcpp::Node> node,
                             std::shared_ptr<RobotLibrary::Control::SerialLinkBase> controller,
                             std::shared_ptr<std::mutex> mutex,
                             const std::string &actionName = "track_joint_trajectory",
                             const std::string &controlTopicName = "joint_commands");
    
    private:
      
        std::vector<serial_link_interfaces::msg::Statistics> _errorStatistics;                      ///< Stored data on position tracking error        
        
        RobotLibrary::Trajectory::SplineTrajectory _trajectory;                                     ///< Trajectory object
  
        /**
         * @brief Processes the request to execute action.
         * @param uuid A unique identification for the goal.
         * @param request The goal component of the action definition.
         * @return REJECT if the arguments are not sound, ACCEPT_AND_EXECUTE otherwise.
         */
        rclcpp_action::GoalResponse
        handle_goal(const rclcpp_action::GoalUUID &uuid,
                    std::shared_ptr<const typename Action::Goal> goal);
        
        /**
         * This is the main control loop for joint trajectory tracking.
         * This is a virtual function and must be defined in any derived class.
         * @param actionManager A pointer to the rclcpp::ServerGoalHandle for this action
         */
        void
        execute(const std::shared_ptr<GoalHandle> actionManager);
        
        /**
         * Completes the action and sends result to the client.
         * @param status 1 = Completed, 2 = Cancelled, 3 = Aborted
         * @param message Information for the client
         */
        void
        cleanup_and_send_result(const int &status,
                                const std::string &message,
                                const std::shared_ptr<GoalHandle> actionManager);
        
};                                                                                                  // Semicolon required after a class declaration
            
#endif
