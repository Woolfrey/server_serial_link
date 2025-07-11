/**
 * @file    hold_configuration.hpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.0
 * @brief   A ROS2 action that enables a robot to hold a given joint configuration indefinitely.
 *
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the control class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */
 
#ifndef HOLD_CONFIGURATION_H
#define HOLD_CONFIGURATION_H

#include <serial_link_action_server/action_server_base.hpp>
#include <serial_link_interfaces/action/hold_configuration.hpp>
#include <serial_link_interfaces/msg/joint_state.hpp>
#include <serial_link_interfaces/msg/statistics.hpp>
 
namespace serial_link_action_server {

/**
 * @brief A class for a ROS2 action that performs continuous feedback control on a set joint configuration.
 */
class HoldConfiguration : public ActionServerBase<serial_link_interfaces::action::HoldConfiguration>
{
    using Action = serial_link_interfaces::action::HoldConfiguration;
    using ActionManager = rclcpp_action::ServerGoalHandle<Action>;                                  // For brevity
    
    public:
    
        /**
         * @brief Constructor.
         * @param node A pointer to a ROS2 node for this action.
         * @param controller A pointer to a RobotLibrary controller for a serial link robot arm.
         * @param mutex Stops 2 actions using a robot at the same time.
         * @param actionName Action name that is advertised on the ROS2 network.
         * @param controlTopicName The topic to which the joint commands will be published.
         */
        HoldConfiguration(std::shared_ptr<rclcpp::Node> node,
                          std::shared_ptr<RobotLibrary::Control::SerialLinkBase> controller,
                          std::shared_ptr<std::mutex> mutex,
                          const std::string &actionName = "hold_configuration",
                          const std::string &controlTopicName = "joint_commands");
    
        private:
        
        Eigen::VectorXd _desiredPosition;
        
        std::vector<serial_link_interfaces::msg::Statistics> _errorStatistics;                      ///< Summary of performance
      
        /**
         * @brief Processes the request to execute action.
         * @param uuid A unique goal identification. Not used.
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
};                                                                                                  // Semicolon needed after class declaration

} // namespace

#endif
