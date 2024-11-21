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
           
        RobotLibrary::CartesianSpline _trajectory;                                                  ///< Trajectory object

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
   
    // Statistics
    serial_link_action_server::msg::Statistics positionError;
    positionError.mean = 0.0;
    positionError.variance = 0.0;
    positionError.min = std::numeric_limits<double>::max();
    positionError.max = std::numeric_limits<double>::lowest();

    serial_link_action_server::msg::Statistics orientationError;
    orientationError.mean = 0.0;
    orientationError.variance = 0.0;
    orientationError.min = std::numeric_limits<double>::max();
    orientationError.max = std::numeric_limits<double>::lowest();  
   
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
    RCLCPP_INFO(_node->get_logger(), "Executing Cartesian trajectory tracking.");                   // Server side info
    
    auto request = actionManager->get_goal();                                                       // Retrieve goal
    
    auto result = std::make_shared<Action::Result>();                                               // Stores the result statistics, message
    
    rclcpp::Rate loopRate(_controller->frequency());                                                // This regulates the control frequency
    
    unsigned long long n = 1;                                                                       // This is used for computing statistics

    rclcpp::Clock timer;
    
    double startTime = timer.now().seconds();

    double elapsedTime = 0.0;
    
    do
    {
        // Check for cancellation
        if(actionManager->is_canceling())
        {
            _padlock->unlock();                                                                     // Release control of robot
            
            result->message = "Cartesian trajectory tracking cancelled.";                           // Message for client

            actionManager->canceled(result);                                                        // Specify result as cancelled
            
            publish_joint_command(Eigen::VectorXd::Zero(_numJoints));                               // Ensure final command is zero
            
            RCLCPP_INFO(_node->get_logger(), "Cartesian trajectory tracking cancelled.");           // Server side info
            
            return;
        }
        
        _controller->update();                                                                      // Updates the Jacobian
        
        elapsedTime = timer.now().seconds() - startTime;                                            // Determine elapsed time
        
        const auto &[desiredPose,
                     desiredVelocity,
                     desiredAcceleration] = _trajectory.query_state(elapsedTime);                   // Get the desired state from the trajectory generator at the given time

        // NOTE: Use try/catch here:
        publish_joint_command(_controller->track_endpoint_trajectory(desiredPose,
                                                                     desiredVelocity,
                                                                     desiredAcceleration));         // Solve the resolved motion rate control algorithm
           
        // Put actual state data in to feedback field
        RobotLibrary::Pose actualPose = _controller->endpoint_pose();
        Eigen::Vector<double,6> twist = _controller->endpoint_velocity();
        
        _feedback->actual.pose.position.x = actualPose.translation()[0];
        _feedback->actual.pose.position.y = actualPose.translation()[1];
        _feedback->actual.pose.position.z = actualPose.translation()[2];
        _feedback->actual.pose.orientation.w = actualPose.quaternion().w();
        _feedback->actual.pose.orientation.x = actualPose.quaternion().x();
        _feedback->actual.pose.orientation.y = actualPose.quaternion().y();
        _feedback->actual.pose.orientation.z = actualPose.quaternion().z();
        
        _feedback->actual.twist.linear.x = twist[0];
        _feedback->actual.twist.linear.y = twist[1];
        _feedback->actual.twist.linear.z = twist[2];
        _feedback->actual.twist.angular.x = twist[3];
        _feedback->actual.twist.angular.y = twist[4];
        _feedback->actual.twist.angular.z = twist[5];  
        
        // Put desired state data in to feedback field
        _feedback->desired.pose.position.x = desiredPose.translation()[0];
        _feedback->desired.pose.position.y = desiredPose.translation()[1];
        _feedback->desired.pose.position.z = desiredPose.translation()[2];
        _feedback->desired.pose.orientation.w = desiredPose.quaternion().w();
        _feedback->desired.pose.orientation.x = desiredPose.quaternion().x();
        _feedback->desired.pose.orientation.y = desiredPose.quaternion().y();
        _feedback->desired.pose.orientation.z = desiredPose.quaternion().z();
        
        _feedback->desired.twist.linear.x  = desiredVelocity[0];
        _feedback->desired.twist.linear.y  = desiredVelocity[1];
        _feedback->desired.twist.linear.z  = desiredVelocity[2];
        _feedback->desired.twist.angular.x = desiredVelocity[3];
        _feedback->desired.twist.angular.y = desiredVelocity[4];
        _feedback->desired.twist.angular.z = desiredVelocity[5];
        
        _feedback->desired.accel.linear.x  = desiredAcceleration[0];
        _feedback->desired.accel.linear.y  = desiredAcceleration[1];
        _feedback->desired.accel.linear.z  = desiredAcceleration[2];
        _feedback->desired.accel.angular.x = desiredAcceleration[3];
        _feedback->desired.accel.angular.y = desiredAcceleration[4];
        _feedback->desired.accel.angular.z = desiredAcceleration[5];  
        
        // Put pose error data in to feedback field
        Eigen::Vector<double,6> error = actualPose.error(desiredPose);
        _feedback->position_error = error.head(3).norm();
        _feedback->orientation_error = error.tail(3).norm(); 
        _feedback->time_remaining = _trajectory.end_time() - elapsedTime;                           // As it says
        
        actionManager->publish_feedback(_feedback);                                                 // Make feedback available over ROS2 network

        loopRate.sleep();                                                                           // Synchronize the control loop

        ++n;                                                                                        // Increment counter
        
    } while (rclcpp::ok() and elapsedTime <= request->points.back().time);

    // Send the result to the client
    if (rclcpp::ok())
    {
        _padlock->unlock();                                                                            // Release control of robot
        
        actionManager->succeed(result);                                                             // Action completed successfully

        RCLCPP_INFO(_node->get_logger(), "Trajectory tracking complete. Awaiting new request.");    // Server side info
    }
}
            
#endif
