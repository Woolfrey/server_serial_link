/**
 * @file   TrackCartesianTrajectory.h
 * @author Jon Woolfrey
 * @data   June 2024
 * @brief  A ROS2 action server for Cartesian trajectory tracking.
 */

#ifndef TRACKCARTESIANTRAJECTORY_H
#define TRACKCARTESIANTRAJECTORY_H

#include <ActionServerBase.h>
#include <RobotLibrary/CartesianSpline.h>                                                           // Trajectory generator
#include "serial_link_action_server/action/track_cartesian_trajectory.hpp"                          // Custom generated action

/**
 * This class performs joint trajectory tracking for a serial link robot arm.
 */
class TrackCartesianTrajectory : public ActionServerBase<serial_link_action_server::action::TrackCartesianTrajectory>
{
    public:

        using Action  = serial_link_action_server::action::TrackCartesianTrajectory;
        using ActionManager = rclcpp_action::ServerGoalHandle<Action>;
                
        /**
         * Constructor for the class.
         * @param A pointer to a robot arm controller.
         * @param options I have no idea what this does ¯\_(ツ)_/¯
         */
        TrackCartesianTrajectory(std::shared_ptr<rclcpp::Node> node,
                                 SerialLinkBase *controller,
                                 std::mutex *mutex,
                                 const std::string &actionName = "track_joint_trajectory",
                                 const std::string &controlTopicName = "joint_commands");
    
    private:
   
        CartesianSpline _trajectory;                                                                ///< Trajectory object

        /**
         * Processes the request to execute action.
         * This is a virtual function and must be defined by any derived class.
         * @param uuid I have no idea what this does ¯\_(ツ)_/¯
         * @param request The goal component of the action definition.
         * @return REJECT if the arguments are not sound, ACCEPT_AND_EXECUTE otherwise.
         */
        virtual
        rclcpp_action::GoalResponse
        request_action(const rclcpp_action::GoalUUID &uuid,
                       std::shared_ptr<const typename Action::Goal> request);
        
        /**
         * Processes the cancel request.
         * This is a virtual function and must be defined in any derived class.
         * @param actionManager A pointer to the rclcpp::ServerGoalHandle for this action
         * @return rclcpp_action::CancelResponse::ACCEPT
         */
        virtual
        rclcpp_action::CancelResponse
        cancel(const std::shared_ptr<ActionManager> actionManager);
        
        /**
         * This is the main control loop for joint trajectory tracking.
         * This is a virtual function and must be defined in any derived class.
         * @param actionManager A pointer to the rclcpp::ServerGoalHandle for this action
         */
        virtual
        void
        execute_action(const std::shared_ptr<ActionManager> actionManager);       
};                                                                                                  // Semicolon required after a class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
TrackCartesianTrajectory::TrackCartesianTrajectory(std::shared_ptr<rclcpp::Node> node,
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
    // Need to do something here
}

  //////////////////////////////////////////////////////////////////////////////////////////////////// 
 //                          Process request to track Cartesian trajectory                         // 
////////////////////////////////////////////////////////////////////////////////////////////////////  
inline                             
rclcpp_action::GoalResponse
TrackCartesianTrajectory::request_action(const rclcpp_action::GoalUUID &uuid,
                                         std::shared_ptr<const Action::Goal> request)
{
    (void)uuid;                                                                                     // Prevents colcon from throwing a warning
    
    // Try to lock the mutex and reject if another action is running
    if (!_mutex->try_lock())
    {
        RCLCPP_WARN(_node->get_logger(), "Request rejected. Another action server is currently running.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    RCLCPP_WARN(_node->get_logger(), "Request for Cartesian control received.");
    
    std::vector<double> times(1, 0.0);                                                              // Initialise time vector with start of 0.0s
    
    std::vector<Pose> poses;                                                                        // Poses that make up the trajectory spline
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
                                         
       if(point.reference == 0)                                                                     // Pose is in the global / base frame
       {
            poses.emplace_back(Pose(translation,quaternion));
       }
       else if(point.reference == 1)                                                                // Pose is in the endpoint frame
       {
            poses.emplace_back(poses.back()*Pose(translation, quaternion));
       }
       else if(point.reference == 2)                                                                // Pose is relative to endpoint frame, but in base frame coordinates
       {
            translation = poses.back().translation() + translation;                                 // Add the translation
            quaternion  = poses.back().quaternion()*quaternion;                                     // Add the rotation
            
            poses.emplace_back(Pose(translation,quaternion));
       }
   }
   
   // Try to create the trajectory
   try
   {
        _trajectory = CartesianSpline(poses,times,_controller->endpoint_velocity());
   }
   catch(const std::exception &exception)
   {
        RCLCPP_ERROR(_node->get_logger(), "Trajectory creation failed: %s", exception.what());
        
        _mutex->unlock();
        
        return rclcpp_action::GoalResponse::REJECT;
    }
   
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;                                         // Return success and continue to execution
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Cancel the action                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
inline
rclcpp_action::CancelResponse
TrackCartesianTrajectory::cancel(const std::shared_ptr<ActionManager> actionManager)
{
    RCLCPP_INFO(_node->get_logger(), "Received request to cancel Cartesian trajectory tracking.");
    
    auto result = std::make_shared<Action::Result>();                                               // Result portion of the action  
            
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
TrackCartesianTrajectory::execute_action(const std::shared_ptr<ActionManager> actionManager)
{
    auto request = actionManager->get_goal();                                                       // Retrieve goal
    auto result = std::make_shared<Action::Result>();                                               // Stores the result statistics, message
    rclcpp::Rate loopRate(_controller->frequency());                                                // This regulates the control frequency
    unsigned long long n = 1;                                                                       // This is used for computing statistics

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
    
    RCLCPP_INFO(_node->get_logger(), "Executing Cartesian trajectory tracking.");
    
    startTime = timer.now().seconds();

    double elapsedTime = 0.0;
    
    do
    {
        _controller->update();                                                                      // Update controller properties (e.g. Jacobian)
        
        elapsedTime = timer.now().seconds() - startTime;                                            // Determine elapsed time
        
        const auto &[desiredPose,
                     desiredVelocity,
                     desiredAcceleration] = _trajectory.query_state(elapsedTime);                   // Get the desired state from the trajectory generator at the given time

        publish_joint_command(_controller->track_endpoint_trajectory(desiredPose,
                                                                     desiredVelocity,
                                                                     desiredAcceleration));         // Solve the resolved motion rate control algorithm
           
        // Put actual state data in to feedback field
        Pose actualPose = _controller->endpoint_pose();
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
        _feedback->position_error     = error.head(3).norm();
        _feedback->orientation_error  = error.tail(3).norm(); 
        
        _feedback->time_remaining = _trajectory.end_time() - elapsedTime;                           // As it says
        
        actionManager->publish_feedback(_feedback);                                                 // Make feedback available over ROS2 network

        loopRate.sleep();                                                                           // Synchronize the control loop

        ++n;                                                                                        // Increment counter
        
    } while (rclcpp::ok() and elapsedTime <= request->points.back().time);

    // Send the result to the client
    if (rclcpp::ok())
    {
        RCLCPP_INFO(_node->get_logger(), "Trajectory tracking complete.");

        result->successful = 1;

        actionManager->succeed(result);

        RCLCPP_INFO(_node->get_logger(), "Awaiting new request.");
        
        _mutex->unlock();
    }
}
            
#endif
