/**
 * @file   TrackJointTrajectory.h
 * @author Jon Woolfrey
 * @data   June 2024
 * @brief  A ROS2 action server for joint trajectory tracking.
 */

#ifndef TRACKJOINTTRAJECTORY_H
#define TRACKJOINTTRAJECTORY_H

#include "ActionServerBase.h"
#include <RobotLibrary/SplineTrajectory.h>
#include "serial_link_action_server/action/track_joint_trajectory.hpp"

/**
 * This class performs joint trajectory tracking for a serial link robot arm.
 */
class TrackJointTrajectory : public ActionServerBase<serial_link_action_server::action::TrackJointTrajectory>
{
    public:
    
        using Action  = serial_link_action_server::action::TrackJointTrajectory;
        using ActionManager = rclcpp_action::ServerGoalHandle<Action>;
                   
        /**
         * Constructor for the class.
         * @param node A shared pointer to a node
         * @param controller A pointer to a robot arm controller
         * @param mutex An object that blocks 2 actions controlling the robot
         * @param actionName The name to be advertised on the ROS2 network
         * @param controlTopicName The name for the joint command publisher
         * @param options I have no idea what this does ¯\_(ツ)_/¯
         */
        TrackJointTrajectory(std::shared_ptr<rclcpp::Node> node,
                             RobotLibrary::SerialLinkBase *controller,
                             std::mutex* padlock,
                             const std::string &actionName = "track_joint_trajectory",
                             const std::string &controlTopicName = "joint_commands");
    
    private:
      
        std::vector<serial_link_action_server::msg::Statistics> _errorStatistics;                   ///< Stored data on position tracking error        
        
        RobotLibrary::SplineTrajectory _trajectory;                                                 ///< Trajectory object
  
        /**
         * Processes the request to execute action.
         * @param uuid I have no idea what this does ¯\_(ツ)_/¯
         * @param request The goal component of the action definition.
         * @return REJECT if the arguments are not sound, ACCEPT_AND_EXECUTE otherwise.
         */
        rclcpp_action::GoalResponse
        process_request(const rclcpp_action::GoalUUID &uuid,
                        std::shared_ptr<const typename Action::Goal> request);
        
        /**
         * This is the main control loop for joint trajectory tracking.
         * This is a virtual function and must be defined in any derived class.
         * @param actionManager A pointer to the rclcpp::ServerGoalHandle for this action
         */
        void
        execute(const std::shared_ptr<ActionManager> actionManager);
        
        /**
         * Completes the action and sends result to the client.
         * @param status 1 = Completed, 2 = Cancelled, 3 = Aborted
         * @param message Information for the client
         */
        void
        cleanup_and_send_result(const int &status,
                                const std::string &message,
                                const std::shared_ptr<ActionManager> actionManager);
        
};                                                                                                  // Semicolon required after a class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
TrackJointTrajectory::TrackJointTrajectory(std::shared_ptr<rclcpp::Node> node,
                                           RobotLibrary::SerialLinkBase *controller,
                                           std::mutex *padlock,
                                           const std::string &actionName,
                                           const std::string &controlTopicName)
                                           : ActionServerBase
                                             (
                                                node,
                                                controller,
                                                padlock,
                                                actionName,
                                                controlTopicName
                                             )
{
    // Set the size of arrays based on number of joints in robot model
    
    _feedback->actual.position.resize(_numJoints);
    _feedback->actual.velocity.resize(_numJoints);
//  _feedback->actual.acceleration.resize(_numJoints);                                              // Not defined for "actual"
    _feedback->actual.effort.resize(_numJoints);
    
    _feedback->desired.position.resize(_numJoints);
    _feedback->desired.velocity.resize(_numJoints);
    _feedback->desired.acceleration.resize(_numJoints);
//  _feedback->desired.effort.resize(_numJoints);                                                   // Not defined for "desired"
    
    _feedback->error.position.resize(_numJoints);
    _feedback->error.velocity.resize(_numJoints);
//  _feedback->error.acceleration.resize(_numJoints);                                               // Not defined for "error"
//  _feedback->error.effort.resize(_numJoints);                                                     // Not defined for "error"
    
    _errorStatistics.resize(_numJoints);                                                            // Data on position tracking error
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Process a request to perform joint trajectory tracking                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
rclcpp_action::GoalResponse
TrackJointTrajectory::process_request(const rclcpp_action::GoalUUID &uuid,
                                      std::shared_ptr<const Action::Goal> request)
{
    (void)uuid;                                                                                     // Stops colcon from throwing a warning

    if(not _padlock->try_lock())
    {
        RCLCPP_WARN(_node->get_logger(), "Request for joint trajectory tracking rejected. "
                                         "Another action is using the robot.");
                    
                
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    std::vector<double> times(1, 0.0);                                                              // Initialize times with the starting point
    
    std::vector<Eigen::VectorXd> positions;                                                         // Waypoints for the trajectory
    
    positions.reserve(request->points.size() + 1);                                                  // Reserve space for efficiency
    
    positions.emplace_back(_controller->model()->joint_positions());                                // Start trajectory from current position

    // Iterate over trajectory points in the request
    for (const auto &point : request->points)
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

    // Initialize error statistics
    for (auto &error : _errorStatistics)
    {
        error.mean = 0.0;
        error.variance = 0.0;
        error.min = std::numeric_limits<double>::max();
        error.max = std::numeric_limits<double>::lowest();
    }

    // Create the trajectory (it could throw an error, so we need to catch it)
    try
    {
        _trajectory = RobotLibrary::SplineTrajectory(positions, times, _controller->model()->joint_velocities());
    }
    catch (const std::exception &exception)
    {
        RCLCPP_ERROR(_node->get_logger(), "Joint trajectory generation failed.\n%s", exception.what());

        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;                                         // Return success and continue to execution
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       MAIN CONTROL LOOP                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////       
void   
TrackJointTrajectory::execute(const std::shared_ptr<ActionManager> actionManager)
{
    RCLCPP_INFO(_node->get_logger(), "Executing joint trajectory tracking.");
    
    auto request = actionManager->get_goal();                                                       // The goal component of the action definition
    
    rclcpp::Rate loopRate(_controller->frequency());                                                // Regulates the control loop to the robot frequency
    
    unsigned long long n = 1;                                                                       // Counter for statistics
    
    rclcpp::Clock timer;                                                                            // Used to query trajectory
    
    double startTime = timer.now().seconds();                                                       // Start timer

    while (rclcpp::ok())
    {
        // Check if the action has been canceled
        if (actionManager->is_canceling())
        {
            cleanup_and_send_result(2, "Joint trajectory tracking cancelled.", actionManager);
            
            return;
        }
        
        double elapsedTime = timer.now().seconds() - startTime;                                     // Time since start of control sequence

        if (elapsedTime > request->points.back().time) break;                                       // Finished

        _controller->update();                                                                      // Updates kinematic properties

        const auto &[desiredPosition,
                     desiredVelocity,
                     desiredAcceleration] = _trajectory.query_state(elapsedTime);                   // Get the desired state for the given time
                     
        Eigen::VectorXd jointCommand;                                                               // We want to compute this
        
        // Controller may throw an error, so we need to catch it
        try
        {
            jointCommand = _controller->track_joint_trajectory(desiredPosition, desiredVelocity, desiredAcceleration);
        }
        catch(const std::exception &exception)
        {
            RCLCPP_ERROR(_node->get_logger(), exception.what());
            
            cleanup_and_send_result(3, "Controller failed to solve control.", actionManager);
            
            return;
        }
                     
        publish_joint_command(jointCommand);                                                        // Send over ROS2 network

        // Update feedback & result
        for (unsigned int j = 0; j < _numJoints; ++j)
        {
            // Put Eigen data in to ROS msg data
            _feedback->desired.position[j] = desiredPosition[j];
            _feedback->desired.velocity[j] = desiredVelocity[j];
            _feedback->desired.acceleration[j] = desiredAcceleration[j];

            _feedback->actual.position[j] = _controller->model()->joint_positions()[j];
            _feedback->actual.velocity[j] = _controller->model()->joint_velocities()[j];

            double positionError = _feedback->desired.position[j] - _feedback->actual.position[j];
            double velocityError = _feedback->desired.velocity[j] - _feedback->actual.velocity[j];

            _feedback->error.position[j] = positionError;
            _feedback->error.velocity[j] = velocityError;

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
        }

        _feedback->time_remaining = _trajectory.end_time() - elapsedTime;                           // As it says
        
        actionManager->publish_feedback(_feedback);                                                 // Publish feedback

        ++n;                                                                                        // Increment counter for computing statistics
        
        loopRate.sleep();                                                                           // Synchronise to the robot control frequency
    }

    cleanup_and_send_result(1, "Joint trajectory tracking completed.", actionManager);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Complete the action and send result to server                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
TrackJointTrajectory::cleanup_and_send_result(const int &status,
                                              const std::string &message,
                                              const std::shared_ptr<ActionManager> actionManager)
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

    _padlock->unlock();                                                                             // Release control in all cases
}
            
#endif
