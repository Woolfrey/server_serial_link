/**
 * @file    track_cartesian_trajectory.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    March 2025
 * @version 1.0
 * @brief   Source files for the TrackCartesianTrajectory class.
 * 
 * @details This class creates & advertises a ROS2 for Cartesian trajectory tracking. Given a set of
 *          poses and times, it generates a spline trajectory. It then performs real-time feedback
 *          control to make the endpoint of the robot arm follow this trajectory.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the control class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include <serial_link_action_server/track_cartesian_trajectory.hpp>

namespace serial_link_action_server {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
TrackCartesianTrajectory::TrackCartesianTrajectory(std::shared_ptr<rclcpp::Node> node,
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
    _feedback->header.frame_id = _controller->model()->base_name();
}

  //////////////////////////////////////////////////////////////////////////////////////////////////// 
 //                          Process request to track Cartesian trajectory                         // 
////////////////////////////////////////////////////////////////////////////////////////////////////  
rclcpp_action::GoalResponse
TrackCartesianTrajectory::handle_goal(const rclcpp_action::GoalUUID &uuid,
                                      std::shared_ptr<const Action::Goal> goal)
{
    (void)uuid;                                                                                     // Prevents colcon from throwing a warning
    
    // Ensure arguments are sound
    if(goal->position_tolerance <= 0.0
    or goal->orientation_tolerance <= 0.0)
    {
        RCLCPP_WARN(_node->get_logger(), "Tolerances were not positive. Position tolerance was %f. Orientation tolerance was %f.",
                    goal->position_tolerance, goal->orientation_tolerance);

        return rclcpp_action::GoalResponse::REJECT;
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
    
    // Set up the trajectory
    std::vector<double> times(1, 0.0);                                                              // Initialise time vector with start of 0.0s
    std::vector<RobotLibrary::Model::Pose> poses;                                                   // Poses that make up the trajectory spline
    poses.reserve(goal->points.size()+1);                                                           // Reserve space for efficiency
    poses.emplace_back(_controller->endpoint_pose());                                               // First pose is current endpoint
    
    // Iterate over the poses in the request
    for(const auto &point : goal->points)
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
            if(point.reference_frame == 0) poses.emplace_back(RobotLibrary::Model::Pose(translation,quaternion));// In base frame
       else if(point.reference_frame == 1) poses.emplace_back(poses.back()*RobotLibrary::Model::Pose(translation,quaternion)); // In current endpoint frame; transform to base
       else if(point.reference_frame == 2)                                                          // Pose is relative to endpoint frame, but in base frame coordinates
       {
            translation = poses.back().translation() + translation;                                 // Add the translation
            quaternion  = poses.back().quaternion()*quaternion;                                     // Add the rotation
            poses.emplace_back(RobotLibrary::Model::Pose(translation,quaternion));
       }
   }
   
    // Try to create the trajectory
    try
    {
        _trajectory = RobotLibrary::Trajectory::CartesianSpline(poses,times,_controller->endpoint_velocity());
    }
    catch(const std::exception &exception)
    {
        RCLCPP_ERROR(_node->get_logger(), "Trajectory creation failed: %s", exception.what());
        
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Make sure no other action is using the robot
    if(not _mutex->try_lock())
    {
        RCLCPP_WARN(_node->get_logger(),
                    "Request for Cartesian trajectory tracking rejected. "
                    "Another action is currently using the robot.");
        
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;                                         // Return success and continue to execution
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       MAIN CONTROL LOOP                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////  
void
TrackCartesianTrajectory::execute(const std::shared_ptr<GoalHandle> goalHandle)
{
    RCLCPP_INFO(_node->get_logger(), "Executing Cartesian trajectory tracking.");                   // Inform user
    
    auto goal = goalHandle->get_goal();                                                             // Save it so we can reference it later
    
    rclcpp::Rate loopRate(_controller->frequency());                                                // Used to regulate control loop timing
    
    unsigned long long int n = 1;                                                                   // Used for statistics    
      
    double startTime = _node->get_clock()->now().seconds();                                         // Used to query trajectory
    
    while(rclcpp::ok())
    {
        // Check to see if the action has been cancelled
        if (goalHandle->is_canceling())
        {
            cleanup_and_send_result(2, "Cartesian trajectory tracking cancelled.", goalHandle);
            return;
        }
        
        double elapsedTime = _node->now().seconds() - startTime;
        
        if(elapsedTime > goal->points.back().time) break;    
        
         _controller->update();                                                                     // Update Jacobian etc.
         
         const auto &[desiredPose,
                      desiredVelocity,
                      desiredAcceleration] = _trajectory.query_state(elapsedTime);
         
        // Controller may throw a runtime error, so we need to catch it
        try
        {
            Eigen::VectorXd jointCommands = _controller->track_endpoint_trajectory(desiredPose, desiredVelocity, desiredAcceleration);
            
            publish_joint_command(jointCommands);                                                   // Send immediately to robot
            
            // Update feedback fields
            RobotLibrary::Model::Pose actualPose = _controller->endpoint_pose();                    // Get computed pose
            RL_pose_to_ROS(_feedback->actual.pose, actualPose);                                     // Convert from RobotLibrary object to ROS2 msg
            
            Eigen::Vector<double,6> twist = _controller->endpoint_velocity();                       // Get computed endpoint velocity
            Eigen_twist_to_ROS(_feedback->actual.twist, twist);                                     // Convert from Eigen object to ROS2 msg
            
            RL_pose_to_ROS(_feedback->desired.pose, desiredPose);
            
            Eigen_twist_to_ROS(_feedback->desired.twist, desiredVelocity);           

            // Set the desired accelerations from the trajectory
            _feedback->desired.accel.linear.x  = desiredAcceleration[0];
            _feedback->desired.accel.linear.y  = desiredAcceleration[1];
            _feedback->desired.accel.linear.z  = desiredAcceleration[2];
            _feedback->desired.accel.angular.x = desiredAcceleration[3];
            _feedback->desired.accel.angular.y = desiredAcceleration[4];
            _feedback->desired.accel.angular.z = desiredAcceleration[5];
            
            _feedback->manipulability = _controller->manipulability();
            
            _feedback->header.stamp = _node->now();
                  
            goalHandle->publish_feedback(_feedback);
            
            // Update error statistics for the result message
            Eigen::Vector<double,6> error = actualPose.error(desiredPose); 
            double positionError = error.head(3).norm();
            double orientationError = error.tail(3).norm();
            update_statistics(_positionError, positionError, n);         
            update_statistics(_orientationError, orientationError, n);
            ++n;                                                                                    // Increment sample size
            
            // Check tolerances
            if (positionError > goal->position_tolerance)
            {
                cleanup_and_send_result(3, "Position error tolerance violated: "
                                        + std::to_string(positionError) + " >= " + std::to_string(goal->position_tolerance) + ".",
                                        goalHandle);
                return;
            }
            else if (orientationError > goal->orientation_tolerance)
            {
                cleanup_and_send_result(3, "Orientation error tolerance violated: "
                                        + std::to_string(orientationError) + " >= " + std::to_string(goal->orientation_tolerance) + ".",
                                        goalHandle);
                return;
            }
              
            loopRate.sleep();                                                                       // Synchronise with control frequency                   
        }
        catch (const std::exception &exception)
        {
            RCLCPP_ERROR(_node->get_logger(), exception.what());
            cleanup_and_send_result(3, "Resolved motion rate control failed.", goalHandle);         // Couldn't solve control; abort.
            return;
        }
    }

    cleanup_and_send_result(1, "Cartesian trajectory tracking completed.", goalHandle);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Completes the action and sends the result to the client                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
TrackCartesianTrajectory::cleanup_and_send_result(const int &status,
                                                  const std::string &message,
                                                  const std::shared_ptr<GoalHandle> goalHandle)
{
    publish_joint_command(Eigen::VectorXd::Zero(_numJoints));                                       // Ensure the last command is zero
            
    // Assign data to the result section of the actions
    auto result = std::make_shared<Action::Result>();                                               // Result portion of the message
    result->position_error = _positionError;
    result->orientation_error = _orientationError;
    result->message = message;
    
    switch(status)
    {
        case 1:                                                                                     // Successfully completed
        {
            goalHandle->succeed(result);
            RCLCPP_INFO(_node->get_logger(), "Cartesian trajectory tracking complete. Awaiting new request.");
            break;
        }
        case 2:                                                                                     // Cancelled
        {
            goalHandle->canceled(result);
            RCLCPP_INFO(_node->get_logger(), "Cartesian trajectory tracking cancelled. Awaiting new request.");
            break;
        }
        case 3:                                                                                     // Aborted
        {
            goalHandle->abort(result);
            RCLCPP_ERROR(_node->get_logger(), "Cartesian trajectory tracking aborted.");
            break;
        }
    }

    _mutex->unlock();                                                                             // Release control
}

}
