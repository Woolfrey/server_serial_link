/**
 * @file   TrackCartesianTrajectory.h
 * @author Jon Woolfrey
 * @data   June 2024
 * @brief  A ROS2 action server for Cartesian trajectory tracking.
 */

#ifndef TRACKCARTESIANTRAJECTORY_H
#define TRACKCARTESIANTRAJECTORY_H

#include "server/ActionServerBase.h"
#include <RobotLibrary/CartesianSpline.h>                                                           // Trajectory generator
#include "serial_link_action_server/action/track_cartesian_trajectory.hpp"                          // Custom generated action

/**
 * This class performs joint trajectory tracking for a serial link robot arm.
 */
class TrackCartesianTrajectory : public ActionServerBase<serial_link_action_server::action::TrackCartesianTrajectory>
{
    public:

        using Action  = serial_link_action_server::action::TrackCartesianTrajectory;                // For brevity       
        
        using ActionManager = rclcpp_action::ServerGoalHandle<Action>;                              // For brevity
                
        /**
         * Constructor for the class.
         * @param node A (shared) pointer to a ROS2 node
         * @param controller A raw pointer to a RobotLibrary controller
         * @param padlock A raw pointer to a mutex for blocking other actions
         * @param actionName What this action will be listed as on the ROS2 network
         * @param controlTopicName For the publisher
         */
        TrackCartesianTrajectory(std::shared_ptr<rclcpp::Node> node,
                                 RobotLibrary::SerialLinkBase *controller,
                                 std::mutex *padlock,
                                 const std::string &actionName = "track_cartesian_trajectory",
                                 const std::string &controlTopicName = "joint_commands");
    
    private:

        serial_link_action_server::msg::Statistics _orientationError;                               ///< Statistical summary of orientation tracking performance
        
        serial_link_action_server::msg::Statistics _positionError;                                  ///< Statistical summary of position tracking performance
       
        RobotLibrary::CartesianSpline _trajectory;                                                  ///< Trajectory generator

        /**
         * Processes the request to execute action.
         * This is a virtual function and must be defined by any derived class.
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
};                                                                                                  // Semicolon required after a class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
TrackCartesianTrajectory::TrackCartesianTrajectory(std::shared_ptr<rclcpp::Node> node,
                                                   RobotLibrary::SerialLinkBase *controller,
                                                   std::mutex *padlock,
                                                   const std::string &actionName,
                                                   const std::string &controlTopicName)
                                                   : ActionServerBase(
                                                        node,
                                                        controller,
                                                        padlock,
                                                        actionName,
                                                        controlTopicName)
{
    // Need to do something here
}

  //////////////////////////////////////////////////////////////////////////////////////////////////// 
 //                          Process request to track Cartesian trajectory                         // 
////////////////////////////////////////////////////////////////////////////////////////////////////  
inline                             
rclcpp_action::GoalResponse
TrackCartesianTrajectory::process_request(const rclcpp_action::GoalUUID &uuid,
                                          std::shared_ptr<const Action::Goal> request)
{
    (void)uuid;                                                                                     // Prevents colcon from throwing a warning

    if(not _padlock->try_lock())
    {
        RCLCPP_WARN(_node->get_logger(),
                    "Request for Cartesian trajectory tracking rejected. "
                    "Another action is currently using the robot.");
        
        return rclcpp_action::GoalResponse::REJECT;
    }
  
    std::vector<double> times(1, 0.0);                                                              // Initialise time vector with start of 0.0s
    
    std::vector<RobotLibrary::Pose> poses;                                                          // Poses that make up the trajectory spline
    
    poses.reserve(request->points.size()+1);                                                        // Reserve space for efficiency
    
    poses.emplace_back(_controller->endpoint_pose());                                               // First pose is current endpoint
    
    // Iterate over the poses in the request
    for(const auto &point : request->points)
    {
        times.push_back(point.time);                                                                // Add the time to the array

        Eigen::Vector3d translation = {point.pose.position.x,
                                       point.pose.position.y,
                                       point.pose.position.z};
                                       
        Eigen::Quaterniond quaternion = {point.pose.orientation.w,
                                         point.pose.orientation.x,
                                         point.pose.orientation.y,
                                         point.pose.orientation.z};
       
       // Check the frame of reference
            if(point.reference == 0) poses.emplace_back(RobotLibrary::Pose(translation,quaternion));// In base frame
       else if(point.reference == 1) poses.emplace_back(poses.back()*RobotLibrary::Pose(translation,quaternion)); // In current endpoint frame; transform to base
       else if(point.reference == 2)                                                                // Pose is relative to endpoint frame, but in base frame coordinates
       {
            translation = poses.back().translation() + translation;                                 // Add the translation
            quaternion  = poses.back().quaternion()*quaternion;                                     // Add the rotation
            poses.emplace_back(RobotLibrary::Pose(translation,quaternion));
       }
   }
   
    // (Re)set statistics
    _positionError.mean = 0.0;
    _positionError.variance = 0.0;
    _positionError.min = std::numeric_limits<double>::max();
    _positionError.max = std::numeric_limits<double>::lowest();

    _orientationError.mean = 0.0;
    _orientationError.variance = 0.0;
    _orientationError.min = std::numeric_limits<double>::max();
    _orientationError.max = std::numeric_limits<double>::lowest();  
   
    // Try to create the trajectory
    try
    {
        _trajectory = RobotLibrary::CartesianSpline(poses,times,_controller->endpoint_velocity());
    }
    catch(const std::exception &exception)
    {
        RCLCPP_ERROR(_node->get_logger(), "Trajectory creation failed: %s", exception.what());
        
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;                                         // Return success and continue to execution
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       MAIN CONTROL LOOP                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////  
void
TrackCartesianTrajectory::execute(const std::shared_ptr<ActionManager> actionManager)
{
    RCLCPP_INFO(_node->get_logger(), "Executing Cartesian trajectory tracking.");

    auto request = actionManager->get_goal();
    auto result = std::make_shared<Action::Result>();
    rclcpp::Rate loopRate(_controller->frequency());

    unsigned long long n = 1;                                                                       // Counter for statistics
    rclcpp::Clock timer;
    double startTime = timer.now().seconds();

    // Inline method
    auto cleanup_and_send_result = [&](const int &status, const std::string &message)
    {
        result->position_error = _positionError;
        result->orientation_error = _orientationError;
        result->message = message;
        
        switch(status)
        {
            case 1: // Successfully completed
            {
                actionManager->succeed(result);
                RCLCPP_INFO(_node->get_logger(), "Cartesian trajectory tracking complete. Awaiting new request.");
                break;
            }
            case 2: // Cancelled
            {
                actionManager->canceled(result);
                publish_joint_command(Eigen::VectorXd::Zero(_numJoints));                           // Zero the command
                RCLCPP_INFO(_node->get_logger(), "Cartesian trajectory tracking cancelled. Awaiting new request.");
                break;
            }
            case 3: // Aborted
            {
                actionManager->abort(result);
                publish_joint_command(Eigen::VectorXd::Zero(_numJoints));
                RCLCPP_ERROR(_node->get_logger(), "Cartesian trajectory tracking aborted.");
                break;
            }
        }

        _padlock->unlock();                                                                         // Release control
    };

    while (rclcpp::ok())
    {
        double elapsedTime = timer.now().seconds() - startTime;

        // Check if the action has been canceled
        if (actionManager->is_canceling())
        {
            cleanup_and_send_result(2, "Cartesian trajectory tracking cancelled.");
            return;
        }

        if (elapsedTime > request->points.back().time)
        {
            break;
        }

        _controller->update();                                                                      // Gets latest Jacobian

        const auto &[desiredPose,
                     desiredVelocity,
                     desiredAcceleration] = _trajectory.query_state(elapsedTime);                   // Query desired state for given time

        Eigen::VectorXd jointCommands = Eigen::VectorXd::Zero(_numJoints);
        
        // QP solver may throw a runtime error, so we need to catch it here.
        try
        {
            jointCommands = _controller->track_endpoint_trajectory(desiredPose, desiredVelocity, desiredAcceleration);
        }
        catch (const std::exception &exception)
        {
            RCLCPP_ERROR(_node->get_logger(), exception.what());
            cleanup_and_send_result(3, "Resolved motion rate control failed.");                     // Abort
            return;
        }
        
        publish_joint_command(jointCommands);                                                       // Send commands over ROS2 network

        // Update feedback fields
        RobotLibrary::Pose actualPose = _controller->endpoint_pose();
        Eigen::Vector<double, 6> twist = _controller->endpoint_velocity();

        auto update_pose_feedback = [](auto &feedbackPose, const RobotLibrary::Pose &pose)
        {
            feedbackPose.position.x = pose.translation()[0];
            feedbackPose.position.y = pose.translation()[1];
            feedbackPose.position.z = pose.translation()[2];
            feedbackPose.orientation.w = pose.quaternion().w();
            feedbackPose.orientation.x = pose.quaternion().x();
            feedbackPose.orientation.y = pose.quaternion().y();
            feedbackPose.orientation.z = pose.quaternion().z();
        };

        auto update_twist_feedback = [](auto &feedbackTwist, const Eigen::Vector<double, 6> &twist)
        {
            feedbackTwist.linear.x = twist[0];
            feedbackTwist.linear.y = twist[1];
            feedbackTwist.linear.z = twist[2];
            feedbackTwist.angular.x = twist[3];
            feedbackTwist.angular.y = twist[4];
            feedbackTwist.angular.z = twist[5];
        };

        update_pose_feedback(_feedback->actual.pose, actualPose);
        update_twist_feedback(_feedback->actual.twist, twist);
        update_pose_feedback(_feedback->desired.pose, desiredPose);
        update_twist_feedback(_feedback->desired.twist, desiredVelocity);

        _feedback->desired.accel.linear.x = desiredAcceleration[0];
        _feedback->desired.accel.linear.y = desiredAcceleration[1];
        _feedback->desired.accel.linear.z = desiredAcceleration[2];
        _feedback->desired.accel.angular.x = desiredAcceleration[3];
        _feedback->desired.accel.angular.y = desiredAcceleration[4];
        _feedback->desired.accel.angular.z = desiredAcceleration[5];

        Eigen::Vector<double, 6> error = actualPose.error(desiredPose);
        _feedback->position_error = error.head(3).norm();
        _feedback->orientation_error = error.tail(3).norm();
        _feedback->time_remaining = _trajectory.end_time() - elapsedTime;

        actionManager->publish_feedback(_feedback);

        loopRate.sleep();
        ++n;
    }

    cleanup_and_send_result(1, "Cartesian trajectory tracking completed.");
}
            
#endif
