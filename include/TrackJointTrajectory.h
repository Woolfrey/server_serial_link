/**
 * @file   TrackJointTrajectory.h
 * @author Jon Woolfrey
 * @data   June 2024
 * @brief  A ROS2 action server for joint trajectory tracking.
 */

#ifndef TRACKJOINTTRAJECTORY_H
#define TRACKJOINTTRAJECTORY_H

#include <Eigen/Dense>                                                                              // Can use Eigen::Map?
#include <mutex>
#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ libraries
#include <rclcpp_action/rclcpp_action.hpp>                                                          // ROS2 Action C++ libraries
#include <RobotLibrary/SerialKinematicControl.h>
#include <RobotLibrary/SplineTrajectory.h>
#include "serial_link_action_server/action/track_joint_trajectory.hpp"

#include <std_msgs/msg/float64_multi_array.hpp>

// Short naming conventions for easier referencing
using JointTrajectoryAction = serial_link_action_server::action::TrackJointTrajectory;
using JointControlManager = rclcpp_action::ServerGoalHandle<serial_link_action_server::action::TrackJointTrajectory>;

/**
 * This class performs joint trajectory tracking for a serial link robot arm.
 */
class TrackJointTrajectory : public rclcpp::Node
{
    public:
        
        /**
         * Constructor for the class.
         * @param A pointer to a robot arm controller.
         * @param options I have no idea what this does ¯\_(ツ)_/¯
         */
        TrackJointTrajectory(SerialKinematicControl *controller,
                             std::mutex *mutex,
                             const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
    private:
        
        std::mutex *_mutex;                                                                         ///< Blocks other actions from controlling same robot
        
        unsigned int _numJoints;                                                                    ///< Number of joints being controlled
            
        rclcpp_action::Server<JointTrajectoryAction>::SharedPtr _actionServer;                      ///< This is the foundation for the class.
        
        std::shared_ptr<JointTrajectoryAction::Feedback> _feedback = std::make_shared<JointTrajectoryAction::Feedback>(); ///< Use this to store feedback
        
        std::vector<serial_link_action_server::msg::Statistics> _errorStatistics;                   ///< Stored data on position tracking error
        
        SerialKinematicControl* _controller;                                                        ///< Pointer to the controller
        
        SplineTrajectory _trajectory;                                                               ///< Trajectory object
  
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _jointControlPublisher;      ///< Joint control topic

        /**
         * Processes the request for the TrackJointTrajectory action.
         * @param uuid I have no idea what this does ¯\_(ツ)_/¯
         * @param request The goal component of the TrackJointControl action definition.
         * @return REJECT if the arguments are not sound, ACCEPT_AND_EXECUTE otherwise.
         */
        inline
        rclcpp_action::GoalResponse
        request_tracking(const rclcpp_action::GoalUUID &uuid,
                         std::shared_ptr<const JointTrajectoryAction::Goal> request);
        
        /**
         * Processes the cancel request.
         * @param actionManager A pointer to the rclcpp::ServerGoalHandle for this action
         * @return rclcpp_action::CancelResponse::ACCEPT
         */
        inline
        rclcpp_action::CancelResponse
        cancel(const std::shared_ptr<JointControlManager> actionManager);
        
        /**
         * This is the main control loop for joint trajectory tracking.
         * @param actionManager A pointer to the rclcpp::ServerGoalHandle for this action
         */
        inline
        void
        track_joint_trajectory(const std::shared_ptr<JointControlManager> actionManager);
        
};                                                                                                  // Semicolon required after a class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
TrackJointTrajectory::TrackJointTrajectory(SerialKinematicControl *controller,
                                           std::mutex *mutex,
                                           const rclcpp::NodeOptions &options)
                                           : Node(controller->model()->name()+"_joint_tracking_server", options),
                                             _mutex(mutex),
                                             _numJoints(controller->model()->number_of_joints()),
                                             _controller(controller)
{
    using namespace std::placeholders;

    _actionServer = rclcpp_action::create_server<JointTrajectoryAction>
    (this, "track_joint_trajectory",
     std::bind(&TrackJointTrajectory::request_tracking, this, _1, _2),
     std::bind(&TrackJointTrajectory::cancel, this, _1),
     std::bind(&TrackJointTrajectory::track_joint_trajectory,this,_1)
    );
 
     _jointControlPublisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_commands", 1); 
     
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
                                                                         
    RCLCPP_INFO(this->get_logger(), "Server initiated. Awaiting action request.");
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Process a request to perform joint trajectory tracking                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
inline
rclcpp_action::GoalResponse
TrackJointTrajectory::request_tracking(const rclcpp_action::GoalUUID &uuid,
                                       std::shared_ptr<const JointTrajectoryAction::Goal> request)
{
    (void)uuid;                                                                                     // Stops colcon from throwing a warning

    RCLCPP_INFO(this->get_logger(), "Request for joint trajectory tracking received.");

    // Try to lock the mutex and reject if another action is running
    if (!_mutex->try_lock())
    {
        RCLCPP_WARN(this->get_logger(), "Request rejected. Another action server is currently running.");
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
            RCLCPP_WARN(this->get_logger(),
                        "Request rejected. Dimensions of trajectory point (%zu) do not match number of joints in model (%u).",
                        point.position.size(), _numJoints);
                        
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
    }
    catch (const std::exception &exception)
    {
        RCLCPP_ERROR(this->get_logger(), "Trajectory creation failed: %s", exception.what());
        
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;                                         // Return success and continue to execution
}
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Cancel a joint trajectory tracking action                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
inline
rclcpp_action::CancelResponse
TrackJointTrajectory::cancel(const std::shared_ptr<JointControlManager> actionManager)
{   

    RCLCPP_INFO(this->get_logger(), "Received request to cancel joint trajectory tracking.");
    
    auto result = std::make_shared<JointTrajectoryAction::Result>();                                // Result portion of the action  
            
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
TrackJointTrajectory::track_joint_trajectory(const std::shared_ptr<JointControlManager> actionManager)
{
    auto request = actionManager->get_goal();                                                       // Retrieve goal
    auto result = std::make_shared<JointTrajectoryAction::Result>();                                   // Stores the result statistics, message
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
        RCLCPP_INFO(this->get_logger(), "Counting down...");
        
        while (rclcpp::ok())
        {
            _feedback->time_remaining = request->delay - (timer.now().seconds() - startTime);
            
            if (_feedback->time_remaining <= 0.0) break;
            
            RCLCPP_INFO(this->get_logger(), "%i", static_cast<int>(_feedback->time_remaining));
            
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }

    // Start trajectory tracking
    
    RCLCPP_INFO(this->get_logger(), "Executing joint trajectory tracking.");
    
    startTime = timer.now().seconds();

    double elapsedTime = 0.0;
    
    do
    {
        _controller->update();
        
        elapsedTime = timer.now().seconds() - startTime;
        
        const auto &[desiredPosition,
                     desiredVelocity,
                     desiredAcceleration] = _trajectory.query_state(elapsedTime);

        Eigen::VectorXd control = _controller->track_joint_trajectory(desiredPosition,
                                                                      desiredVelocity,
                                                                      desiredAcceleration);
        
        std_msgs::msg::Float64MultiArray msg;   
        msg.data = {control.data(), control.data() + control.size()};
        
        _jointControlPublisher->publish(msg);

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

        _feedback->time_remaining = _trajectory.end_time() - elapsedTime;
        
        actionManager->publish_feedback(_feedback);

        loopRate.sleep();                                                                           // Synchronize

        ++n;                                                                                        // Increment counter
        
    } while (rclcpp::ok() and elapsedTime <= request->points.back().time);

    // Send the result to the client
    if (rclcpp::ok())
    {
        RCLCPP_INFO(this->get_logger(), "Trajectory tracking complete.");

        result->position_error = _errorStatistics;
        result->successful = 1;

        actionManager->succeed(result);

        RCLCPP_INFO(this->get_logger(), "Awaiting new request.");
        
        _mutex->unlock();
    }
}
            
#endif
