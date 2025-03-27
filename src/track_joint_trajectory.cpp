/**
 * @file    track_joint_trajectory.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    March 2025
 * @version 1.0
 * @brief   Source files for the TrackJointTrajectory class
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

#include <serial_link_action_server/track_joint_trajectory.hpp>

namespace serial_link_action_server {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
TrackJointTrajectory::TrackJointTrajectory(std::shared_ptr<rclcpp::Node> node,
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
    // Set the size of arrays based on number of joints in robot model
    
    _feedback->actual.position.resize(_numJoints);
    _feedback->actual.velocity.resize(_numJoints);
//  _feedback->actual.acceleration.resize(_numJoints);                                              // Not defined for "actual"
    
    _feedback->desired.position.resize(_numJoints);
    _feedback->desired.velocity.resize(_numJoints);
    _feedback->desired.acceleration.resize(_numJoints);
    
    _errorStatistics.resize(_numJoints);                                                            // Data on position tracking error
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Process a request to perform joint trajectory tracking                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
rclcpp_action::GoalResponse
TrackJointTrajectory::handle_goal(const rclcpp_action::GoalUUID &uuid,
                                  std::shared_ptr<const Action::Goal> goal)
{
    (void)uuid;                                                                                     // Stops colcon from throwing a warning
    
    // Check tolerances
    if(goal->tolerances.size() != _numJoints)
    {
        RCLCPP_WARN(_node->get_logger(), "Tolerance on joint tracking error either not set, or set incorrectly.");
        
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Set up the trajectory
    std::vector<double> times(1, 0.0);                                                              // Initialize times with the starting point
    
    std::vector<Eigen::VectorXd> positions;                                                         // Waypoints for the trajectory
    
    positions.reserve(goal->points.size() + 1);                                                     // Reserve space for efficiency
    
    positions.emplace_back(_controller->model()->joint_positions());                                // Start trajectory from current position

    // Iterate over trajectory points in the request
    for (const auto &point : goal->points)
    {   
        // Check dimension mismatch
        if (point.position.size() != _numJoints)
        {
            RCLCPP_WARN(_node->get_logger(),
                        "Request for joint trajectory tracking rejected. "
                        "Dimensions of trajectory point (%zu) do not match number of joints in model (%u).",
                        point.position.size(), _numJoints);

            return rclcpp_action::GoalResponse::REJECT;
        }

        times.push_back(point.time);                                                                // Add time to the vector

        positions.emplace_back(Eigen::Map<const Eigen::VectorXd>(point.position.data(), point.position.size()));
    }
    
    // Create the trajectory (it could throw an error, so we need to catch it)
    try
    {
        _trajectory = RobotLibrary::Trajectory::SplineTrajectory(positions, times, _controller->model()->joint_velocities());
    }
    catch (const std::exception &exception)
    {
        RCLCPP_ERROR(_node->get_logger(), "Joint trajectory generation failed.\n%s", exception.what());

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
TrackJointTrajectory::execute(const std::shared_ptr<GoalHandle> goalHandle)
{
    RCLCPP_INFO(_node->get_logger(), "Executing joint trajectory tracking.");                       // Inform user
    
    rclcpp::Rate loopRate(_controller->frequency());                                                // Regulate this loop to the robot control frequency
    
    auto goal = goalHandle->get_goal();                                                             // Save this so we can reference it later
    
    unsigned long long n = 1;                                                                       // Sample size for statistics
    
    double startTime = _node->get_clock()->now().seconds();                                         // Save this
    
    while(rclcpp::ok())
    {
        // Check to see if canceling
        if(goalHandle->is_canceling())
        {
            cleanup_and_send_result(2, "Joint trajectory tracking cancelled.", goalHandle);
            return;
        }
        
        double elapsedTime = _node->get_clock()->now().seconds() - startTime;                       // Do I need to explain this?
        
        if(elapsedTime > goal->points.back().time) break;                                           // Reached the end of the trajectory.
        
        _controller->update();                                                                      // Update kinematic properties
        
        const auto &[desiredPosition, desiredVelocity, desiredAcceleration] = _trajectory.query_state(elapsedTime);
        
        // NOTE: Controller may throw a runtime error, so need to catch it here.
        try
        {
            Eigen::VectorXd jointCommand = _controller->track_joint_trajectory(desiredPosition, desiredVelocity, desiredAcceleration);
            
            publish_joint_command(jointCommand);                                                    // Send immediately to the robot                                                  
            
            // Update feedback & result
            for (unsigned int j = 0; j < _numJoints; ++j)
            {
                // Put Eigen data in to ROS msg data
                _feedback->desired.position[j]     = desiredPosition[j];
                _feedback->desired.velocity[j]     = desiredVelocity[j];
                _feedback->desired.acceleration[j] = desiredAcceleration[j];

                _feedback->actual.position[j] = _controller->model()->joint_positions()[j];
                _feedback->actual.velocity[j] = _controller->model()->joint_velocities()[j];
                
                double positionError = _feedback->desired.position[j] - _feedback->actual.position[j];

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

    cleanup_and_send_result(1, "Joint trajectory tracking completed.", goalHandle);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Complete the action and send result to server                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
TrackJointTrajectory::cleanup_and_send_result(const int &status,
                                              const std::string &message,
                                              const std::shared_ptr<GoalHandle> actionManager)
{
    publish_joint_command(Eigen::VectorXd::Zero(_numJoints));                                       // Ensure the last command is zero
    
    auto result = std::make_shared<Action::Result>();                                               // The result component of the action definition
    
    result->position_error = _errorStatistics;                                                      // Assign statistical summary
    
    result->message = message;                                                                      // Assign message

    if (status == 1)                                                                                // Success
    {
        actionManager->succeed(result);
        RCLCPP_INFO(_node->get_logger(), "Joint trajectory tracking complete. Awaiting new request.");
    }
    else if (status == 2)                                                                           // Cancelled
    {
        actionManager->canceled(result);
        RCLCPP_INFO(_node->get_logger(), "Joint trajectory tracking cancelled.");
    }
    else  // status == 3 (aborted)
    {
        actionManager->abort(result);
        RCLCPP_INFO(_node->get_logger(), "Joint trajectory tracking aborted.");
    }

    _mutex->unlock();                                                                             // Release control in all cases
}

}
