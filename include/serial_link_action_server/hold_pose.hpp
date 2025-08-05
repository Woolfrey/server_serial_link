/**
 * @file    hold_pose.hpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.0.0
 * @brief   A ROS2 action that enables a robot to hold a given Cartesian endpoint pose indefinitely.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the control class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */
 
#ifndef HOLD_POSE_H
#define HOLD_POSE_H

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial_link_action_server/action_server_base.hpp>
#include <serial_link_action_server/utilities.hpp>
#include <serial_link_interfaces/action/hold_pose.hpp>                                              // Custom generated action
#include <visualization_msgs/msg/marker_array.hpp>

namespace serial_link_action_server {

/**
 * @brief This class performs joint trajectory tracking for a serial link robot arm.
 */
class HoldPose : public ActionServerBase<serial_link_interfaces::action::HoldPose>
{
    public:

        using Action  = serial_link_interfaces::action::HoldPose;                                   // For brevity       
        
        using ActionManager = rclcpp_action::ServerGoalHandle<Action>;                              // For brevity
                
        /**
         * @brief Constructor for the class.
         * @param node A (shared) pointer to a ROS2 node
         * @param controller A shared pointer to a RobotLibrary controller
         * @param mutex A shared pointer to a mutex for blocking other actions
         * @param actionName What this action will be listed as on the ROS2 network
         * @param controlTopicName For the publisher
         */
        HoldPose(std::shared_ptr<rclcpp::Node> node,
                 std::shared_ptr<RobotLibrary::Control::SerialLinkBase> controller,
                 std::shared_ptr<std::mutex> mutex,
                 const std::string &actionName = "hold_pose",
                 const std::string &controlTopicName = "joint_commands");
    
    private:

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _posePublisher;          ///< Visualises the desired pose
        
        RobotLibrary::Model::Pose _desiredPose;                                                     ///< As it says
        
        serial_link_interfaces::msg::Statistics _orientationError;                                  ///< Statistical summary of orientation tracking performance
        
        serial_link_interfaces::msg::Statistics _positionError;                                     ///< Statistical summary of position tracking performance
       
        visualization_msgs::msg::Marker _arrowMarker;                                               ///< Default properties for arrows
        
        visualization_msgs::msg::MarkerArray _poseMarker;                                           ///< Stores desired pose
        
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
