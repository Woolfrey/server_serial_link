/**
 * @file   TrackJointTrajectory.h
 * @author Jon Woolfrey
 * @data   June 2024
 * @brief  A ROS2 action server for joint trajectory tracking.
 */

#ifndef TRACKJOINTTRAJECTORY_H
#define TRACKJOINTTRAJECTORY_H

#include <ActionServerBase.h>
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
         * @param A pointer to a robot arm controller.
         * @param options I have no idea what this does ¯\_(ツ)_/¯
         */
        TrackJointTrajectory(std::shared_ptr<rclcpp::Node> node,
                             SerialLinkBase *controller,
                             std::mutex *mutex,
                             const std::string &actionName = "track_joint_trajectory",
                             const std::string &controlTopicName = "joint_commands");
    
    private:
       
        std::vector<serial_link_action_server::msg::Statistics> _errorStatistics;                   ///< Stored data on position tracking error        
        
        SplineTrajectory _trajectory;                                                               ///< Trajectory object
  
        /**
         * Processes the request to execute action.
         * This is a virtual function and must be defined by any derived class.
         * @param uuid I have no idea what this does ¯\_(ツ)_/¯
         * @param request The goal component of the action definition.
         * @return REJECT if the arguments are not sound, ACCEPT_AND_EXECUTE otherwise.
         */
        rclcpp_action::GoalResponse
        request_action(const rclcpp_action::GoalUUID &uuid,
                       std::shared_ptr<const typename Action::Goal> request);
        
        /**
         * Processes the cancel request.
         * This is a virtual function and must be defined in any derived class.
         * @param actionManager A pointer to the rclcpp::ServerGoalHandle for this action
         * @return rclcpp_action::CancelResponse::ACCEPT
         */
        rclcpp_action::CancelResponse
        cancel(const std::shared_ptr<ActionManager> actionManager);
        
        /**
         * This is the main control loop for joint trajectory tracking.
         * This is a virtual function and must be defined in any derived class.
         * @param actionManager A pointer to the rclcpp::ServerGoalHandle for this action
         */
        void
        execute_action(const std::shared_ptr<ActionManager> actionManager);  
        
};                                                                                                  // Semicolon required after a class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
TrackJointTrajectory::TrackJointTrajectory(std::shared_ptr<rclcpp::Node> node,
                                           SerialLinkBase *controller,
                                           std::mutex *mutex,
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
    _feedback->actual.effort.resize(_numJoints);
    
    _feedback->desired.position.resize(_numJoints);
    _feedback->desired.velocity.resize(_numJoints);
    _feedback->desired.acceleration.resize(_numJoints);                                             // Not available for desired joint state
    
    _feedback->error.position.resize(_numJoints);
    _feedback->error.velocity.resize(_numJoints);
    
    _errorStatistics.resize(_numJoints);                                                            // Data on position tracking error
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Process a request to perform joint trajectory tracking                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
inline
rclcpp_action::GoalResponse
TrackJointTrajectory::request_action(const rclcpp_action::GoalUUID &uuid,
                                     std::shared_ptr<const Action::Goal> request)
{
    (void)uuid;                                                                                     // Stops colcon from throwing a warning

    RCLCPP_INFO(_node->get_logger(), "Request for joint trajectory tracking received.");

    // Try to lock the mutex and reject if another action is running
    if (not _mutex->try_lock())
    {
        RCLCPP_WARN(_node->get_logger(), "Request rejected. Another action server is currently running.");
        
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
                        "Request rejected. Dimensions of trajectory point (%zu) do not match number of joints in model (%u).",
                        point.position.size(), _numJoints);
            
            _mutex->unlock();
            
            return rclcpp_action::GoalResponse::REJECT;
        }

        times.push_back(point.time);                                                                // Add time to the vector

        positions.emplace_back(Eigen::Map<const Eigen::VectorXd>(point.position.data(),
                                                                 point.position.size()));
    }

    // Create the trajectory
    try
    {
        _trajectory = SplineTrajectory(positions, times, _controller->model()->joint_velocities());
            RCLCPP_INFO(_node->get_logger(), "Trajectory created successfully.");
    }
    catch (const std::exception &exception)
    {
        RCLCPP_ERROR(_node->get_logger(), "Trajectory creation failed: %s", exception.what());
        
        _mutex->unlock();
        
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;                                         // Return success and continue to execution
}
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Cancel a joint trajectory tracking action                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
inline
rclcpp_action::CancelResponse
TrackJointTrajectory::cancel(const std::shared_ptr<ActionManager> actionManager)
{   

    RCLCPP_INFO(_node->get_logger(), "Received request to cancel joint trajectory tracking.");
    
    auto result = std::make_shared<Action::Result>();                                // Result portion of the action  
            
    result->successful = -2;                                                                        // CANCELLED
    
    actionManager->canceled(result);                                                                // Put the result in 
    
    _mutex->unlock();                                                                               // So other actions may be executed

    return rclcpp_action::CancelResponse::ACCEPT;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       MAIN CONTROL LOOP                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////       
inline
void
TrackJointTrajectory::execute_action(const std::shared_ptr<ActionManager> actionManager)
{
    RCLCPP_INFO(_node->get_logger(), "Starting.");
    
    auto request = actionManager->get_goal();                                                       // Retrieve goal
    auto result = std::make_shared<Action::Result>();                                               // Stores the result statistics, message
    rclcpp::Rate loopRate(_controller->frequency());                                                // This regulates the control frequency
    unsigned long long n = 1;                                                                       // This is used for computing statistics

    // Initialize error statistics
    for (auto &error : _errorStatistics)
    {
        error.mean = 0.0;
        error.variance = 0.0;
        error.min = std::numeric_limits<double>::max();
        error.max = std::numeric_limits<double>::lowest();
    }

    rclcpp::Clock timer;
    double startTime = timer.now().seconds();

    // Handle delay before starting trajectory tracking
    if (request->delay > 0.0)
    {
        RCLCPP_INFO(_node->get_logger(), "Counting down...");
        
        while (rclcpp::ok())
        {
            _feedback->time_remaining = request->delay - (timer.now().seconds() - startTime);
            
            if (_feedback->time_remaining <= 0.0) break;
            
            RCLCPP_INFO(_node->get_logger(), "%i", static_cast<int>(_feedback->time_remaining));
            
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }

    // Start trajectory tracking
    
    RCLCPP_INFO(_node->get_logger(), "Executing joint trajectory tracking.");
    
    startTime = timer.now().seconds();

    double elapsedTime = 0.0;
    
    do
    {
        _controller->update();                                                                      // Re-computes forward kinematics & inverse dynamics
        
        elapsedTime = timer.now().seconds() - startTime;                                            // As it says
        
        const auto &[desiredPosition,
                     desiredVelocity,
                     desiredAcceleration] = _trajectory.query_state(elapsedTime);                   // Query the trajectory for the current time

        publish_joint_command(_controller->track_joint_trajectory(desiredPosition,
                                                                      desiredVelocity,
                                                                      desiredAcceleration));        // Solve the feedforward/feedback control

        // Update feedback and error statistics
        for (unsigned int j = 0; j < _numJoints; ++j)
        {
            // Update desired state in feedback
            _feedback->desired.position[j]     = desiredPosition[j];
            _feedback->desired.velocity[j]     = desiredVelocity[j];
            _feedback->desired.acceleration[j] = desiredAcceleration[j];

            // Update actual state in feedback
            _feedback->actual.position[j] = _controller->model()->joint_positions()[j];
            _feedback->actual.velocity[j] = _controller->model()->joint_velocities()[j];

            // Compute and update error
            double positionError = _feedback->desired.position[j] - _feedback->actual.position[j];
            double velocityError = _feedback->desired.velocity[j] - _feedback->actual.velocity[j];

            _feedback->error.position[j] = positionError;
            _feedback->error.velocity[j] = velocityError;

            // Update performance statistics
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
        
        actionManager->publish_feedback(_feedback);                                                 // Make feedback available on ROS2 network

        loopRate.sleep();                                                                           // Synchronize

        ++n;                                                                                        // Increment counter for statistics
        
    } while (rclcpp::ok() and elapsedTime <= request->points.back().time);

    _controller->clear_last_solution();                                                             // Clear the last solution in the QP solver
    
    // Send the result to the client
    if (rclcpp::ok())
    {
        RCLCPP_INFO(_node->get_logger(), "Trajectory tracking complete.");

        result->position_error = _errorStatistics;
        result->successful = 1;

        actionManager->succeed(result);

        RCLCPP_INFO(_node->get_logger(), "Awaiting new request.");
        
        _mutex->unlock();
    }
}
            
#endif
