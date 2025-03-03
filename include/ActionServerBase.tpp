/**
 * @file    ActionServerBase.tpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   Provides a common structure for all action servers.
 * 
 * @details These methods are generic to all types of actions.
 *          Virtual methods for unique actions must be elaborated on in derived classes.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the control class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */
 
#include <ActionServerBase.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
ActionServerBase<Action>::ActionServerBase(std::shared_ptr<rclcpp::Node> node,
                                           std::shared_ptr<RobotLibrary::Control::SerialLinkBase> controller,
                                           std::shared_ptr<std::mutex> mutex,
                                           const std::string &actionName,
                                           const std::string &controlTopicName)
                                           : _node(node),
                                             _numJoints(controller->model()->number_of_joints()),
                                             _controller(controller),
                                             _mutex(mutex)
{
    using namespace std::placeholders;                                                              // For brevity

    // Initialise the action server, link methods
    _actionServer = rclcpp_action::create_server<Action>
    (
        _node,
        actionName,
        std::bind(&ActionServerBase::handle_goal, this, _1, _2),
        std::bind(&ActionServerBase::handle_cancel, this, _1),
        std::bind(&ActionServerBase::handle_accepted,this,_1)
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
 //                             Spin off a thread to execute the action                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
void
ActionServerBase<Action>::handle_accepted(const std::shared_ptr<GoalHandle> goalHandle)
{
    (void) goalHandle;                                                                              // This stops colcon from issuing a warning
    
    std::thread{std::bind(&ActionServerBase::execute, this, std::placeholders::_1), goalHandle}.detach();
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Cancel an action that is running                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
rclcpp_action::CancelResponse
ActionServerBase<Action>::handle_cancel(const std::shared_ptr<GoalHandle> goalHandle)
{
    if (goalHandle->is_active())
    {
        RCLCPP_INFO(_node->get_logger(), "Canceling action.");
        
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    else if (goalHandle->is_canceling())
    {
        RCLCPP_WARN(_node->get_logger(), "Cancel request already received & accepted.");
        
        return rclcpp_action::CancelResponse::REJECT;
    }
    
    RCLCPP_WARN(_node->get_logger(), "Cancel request rejected: Goal is not active or already canceled.");
    return rclcpp_action::CancelResponse::REJECT;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                  Publish a joint command                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
void
ActionServerBase<Action>::publish_joint_command(const Eigen::VectorXd &command)
{
    _jointCommand.header.stamp = _node->now();                                                      // Add time of publication
    
    _jointCommand.command = {command.data(), command.data() + command.size()};                      // Tansfer data
    
    _jointCommandPublisher->publish(_jointCommand);                                                 // Publish control topic
}

