/**
 * @file    hold_configuration.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.0
 * @brief   A ROS2 action that enables a robot to hold a given joint configuration indefinitely.
 *
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the control class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */
 
#include <serial_link_action_server/hold_configuration.hpp>
 
namespace serial_link_action_server {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
HoldConfiguration::HoldConfiguration(std::shared_ptr<rclcpp::Node> node,
                                     std::shared_ptr<RobotLibrary::Control::SerialLinkBase> controller,
                                     std::shared_ptr<std::mutex> mutex,
                                     const std::string &actionName,
                                     const std::string &controlTopicName)
: ActionServerBase(node,
                   controller,
                   mutex,
                   actionName,
                   controlTopicName)
{
    // Resize arrays based on number of joints
    _errorStatistics.resize(_numJoints);                                                            // Data on position tracking error
    
    _feedback->position_error.resize(_numJoints);
    
    _feedback->joint_names.resize(_numJoints);
    
    for (unsigned int i = 0; i < _numJoints; ++i)
    {
        _feedback->joint_names[i] = _controller->model()->joint(i).name();
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Process a request to perform joint trajectory tracking                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
rclcpp_action::GoalResponse
HoldConfiguration::handle_goal(const rclcpp_action::GoalUUID &uuid,
                               std::shared_ptr<const Action::Goal> goal)
{
    (void)uuid;                                                                                     // Stops colcon from throwing a warning
    
    // Check for correct size of tolerances
    if(goal->tolerances.size() < _numJoints)
    {
        RCLCPP_WARN(_node->get_logger(), "Insufficient number of joint tolerances set for this robot.");
        
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Assign the goal configuration
    if (goal->configuration.size() == 0)
    {
        _desiredPosition = _controller->model()->joint_positions();                                 // No desired configuration given; use current.
    }
    else if (goal->configuration.size() == _numJoints)
    {
        _desiredPosition.resize(_numJoints);
       
        for (unsigned int i = 0; i < _numJoints; ++i)
        {
            _desiredPosition[i] = goal->configuration[i];
        }
    }
    else
    {
        RCLCPP_WARN(_node->get_logger(), "Incorrect size for goal configuration. "
                                         "Size must be either 0 (for current configuration), "
                                         "or %u, but reeceived %zu.",
                                         _numJoints, goal->configuration.size());

        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Initialize error statistics
    for (auto &error : _errorStatistics)
    {
        error.mean = 0.0;
        error.variance = 0.0;
        error.min = std::numeric_limits<double>::max();
        error.max = std::numeric_limits<double>::lowest();
    }

    // Ensure robot is not in use
    if(not _mutex->try_lock())
    {
        RCLCPP_WARN(_node->get_logger(), "Request for joint trajectory tracking rejected. "
                                         "Another action is using the robot.");
                    
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;                                         // Return success and continue to execution
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       MAIN CONTROL LOOP                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////       
void   
HoldConfiguration::execute(const std::shared_ptr<GoalHandle> goalHandle)
{
    RCLCPP_INFO(_node->get_logger(), "Holding joint configuration.");                               // Inform user
    
    rclcpp::Rate loopRate(_controller->frequency());                                                // Regulate this loop to the robot control frequency
    
    auto goal = goalHandle->get_goal();                                                             // Save this so we can reference it later
    
    unsigned long long n = 1;                                                                       // Sample size for statistics

    while(rclcpp::ok())
    {
        // Check to see if canceling
        if(goalHandle->is_canceling())
        {
            cleanup_and_send_result(2, "Hold joint configuration cancelled.", goalHandle);
            return;
        }

        // NOTE: Controller may throw a runtime error, so need to catch it here.
        try
        {
            // Solve control and send immediately to robot
            publish_joint_command(_controller->track_joint_trajectory(_desiredPosition,
                                                                      Eigen::VectorXd::Zero(_numJoints),
                                                                      Eigen::VectorXd::Zero(_numJoints)));
            // Update feedback & result
            for (unsigned int j = 0; j < _numJoints; ++j)
            {
                double positionError = _desiredPosition[j] - _controller->model()->joint_positions()[j];
                
                _feedback->position_error[j] = positionError;                                       // Add to feedback field

                // Update statistics
                auto &stats = _errorStatistics[j];
                stats.min = std::min(stats.min, positionError);
                stats.max = std::max(stats.max, positionError);

                if (n > 1)
                {
                    stats.mean = ((n - 1) * stats.mean + positionError) / n;
                    double delta = positionError - stats.mean;
                    stats.variance = ((n - 2) * stats.variance + (n / (n - 1)) * delta * delta) / (n - 1);
                }
                
                if(abs(positionError) > goal->tolerances[j])
                {
                    cleanup_and_send_result(3, "Tolerance for joint " + std::to_string(j) + " violated.", goalHandle); // Abort
                    return;
                }
            }
            
            _feedback->header.stamp = _node->now();
            
            goalHandle->publish_feedback(_feedback);                                                // Send feedback over network
            
            ++n;
            
            loopRate.sleep();
        }
        catch(const std::exception &exception)
        {
            RCLCPP_ERROR(_node->get_logger(), exception.what());                                    // Inform user
            cleanup_and_send_result(3, "Failed to solve control.", goalHandle);                     // Abort  
            return;
        }
    }

    // Technically, this should never be called:
    cleanup_and_send_result(1, "Hold joint configuration completed.", goalHandle);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Complete the action and send result to server                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
HoldConfiguration::cleanup_and_send_result(const int &status,
                                           const std::string &message,
                                           const std::shared_ptr<GoalHandle> actionManager)
{
    publish_joint_command(Eigen::VectorXd::Zero(_numJoints));                                       // Ensure the last command is zero
    
    auto result = std::make_shared<Action::Result>();                                               // The result component of the action definition
    result->position_error = _errorStatistics;                                                      // Assign statistical summary
    result->message= message;                                                                       // Assign message

    if (status == 1)                                                                                // Success
    {
        actionManager->succeed(result);
        RCLCPP_INFO(_node->get_logger(), "Hold configuration complete.");
    }
    else if (status == 2)                                                                           // Cancelled
    {
        actionManager->canceled(result);
        RCLCPP_INFO(_node->get_logger(), "Hold configuration cancelled.");
    }
    else  // status == 3 (aborted)
    {
        actionManager->abort(result);
        RCLCPP_INFO(_node->get_logger(), "Hold joint configuration aborted.");
    }

    _mutex->unlock();                                                                             // Release control in all cases
}

} // namespace

