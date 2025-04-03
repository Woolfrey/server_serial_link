/**
 * @file    follow_transform.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    April 2025
 * @version 1.0
 * @brief   Source code for the FollowTransform action server.
 * 
 * @details This class is the implementation of the FollowTransform action server. The action server
 *          will look up a specified transform and move the robot endpoint so that they are aligned.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the control class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include <serial_link_action_server/follow_transform.hpp>


namespace serial_link_action_server {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
FollowTransform::FollowTransform(std::shared_ptr<rclcpp::Node> node,
                                 std::shared_ptr<RobotLibrary::Control::SerialLinkBase> controller,
                                 std::shared_ptr<std::mutex> mutex,
                                 const std::string &actionName,
                                 const std::string &controlTopicName)
                         
: ActionServerBase(node, controller, mutex, actionName, controlTopicName),
  _transformBuffer(node->get_clock()),                                                              // Pass the node's clock to Buffer
  _transformListener(_transformBuffer)                                                              // Attach listener to buffer
{
    // I need to do something here
}

  //////////////////////////////////////////////////////////////////////////////////////////////////// 
 //                          Process request to track Cartesian trajectory                         // 
////////////////////////////////////////////////////////////////////////////////////////////////////  
rclcpp_action::GoalResponse
FollowTransform::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Action::Goal> goal)
{
    (void)uuid;                                                                                     // Prevents colcon from throwing a warning
    
    RCLCPP_INFO(_node->get_logger(), "Received request to follow transform.");                      // Inform the user
        
    // Ensure arguments are sound 
    if(goal->position_tolerance <= 0.0 or goal->orientation_tolerance <= 0.0)
    {
        RCLCPP_WARN(_node->get_logger(),
                    "Tolerances must be positive. Position tolerance was %f, and orientation tolerance was %f.",
                    goal->position_tolerance, goal->orientation_tolerance);
        
        return rclcpp_action::GoalResponse::REJECT;
    }
    else if(goal->timeout <= 0.0)
    {
        RCLCPP_WARN(_node->get_logger(),
                    "Timeout was %f seconds, but must be positive.",
                    goal->timeout);
        
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
    
    // Make sure no other action is using the robot
    if(not _mutex->try_lock())
    {
        RCLCPP_WARN(_node->get_logger(), "Another action is currently using the robot.");
        
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;                                         // Return success and continue to execution
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       MAIN CONTROL LOOP                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////  
void
FollowTransform::execute(const std::shared_ptr<GoalHandle> goalHandle)
{
    auto goal = goalHandle->get_goal();                                                             // Save it so we can reference it later

    RCLCPP_INFO(_node->get_logger(), "Following the '%s' transform ", goal->frame_id.c_str());      // Inform the user
      
    rclcpp::Rate loopRate(_controller->frequency());                                                // Used to regulate control loop timing
    
    unsigned long long int n = 1;                                                                   // Used for statistics    
    
    while(rclcpp::ok())
    {
        // Check for
        if (goalHandle->is_canceling())
        {
            cleanup_and_send_result(2, "Follow twist cancelled.", goalHandle);
            return;
        }
 
        // Try and look up the transform
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = _transformBuffer.lookupTransform(_controller->model()->base_name(), goal->frame_id, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &exception)
        {
            RCLCPP_ERROR(_node->get_logger(), "Failed to look up transform: %s", exception.what());
            cleanup_and_send_result(3, exception.what(), goalHandle);                               // Abort
            return;
        }
        
        // Convert to RobotLibrary object
        Eigen::Vector3d position(transform.transform.translation.x,
                                 transform.transform.translation.y,
                                 transform.transform.translation.z);

        Eigen::Quaterniond orientation(transform.transform.rotation.w,
                                       transform.transform.rotation.x,
                                       transform.transform.rotation.y,
                                       transform.transform.rotation.z);

        RobotLibrary::Model::Pose desiredPose(position, orientation);
        
        _controller->update();                                                                      // Compute new Jacobian
        
        // Controller could throw an error, so we need to catch it
        try
        {
            Eigen::VectorXd jointCommands = _controller->track_endpoint_trajectory
            (   
                desiredPose,
                Eigen::Vector<double,6>::Zero(),
                Eigen::Vector<double,6>::Zero()
            );
            
            publish_joint_command(jointCommands);                                                   // Send to robot immediately
            
            // Update feedback fields
            RobotLibrary::Model::Pose actualPose = _controller->endpoint_pose();                    // Get computed pose
            
            RL_pose_to_ROS(_feedback->actual.pose, actualPose);                                     // Convert from RobotLibrary object to ROS2 msg
            
            Eigen::Vector<double,6> twist = _controller->endpoint_velocity();                       // Get computed endpoint velocity
            
            Eigen_twist_to_ROS(_feedback->actual.twist, twist);                                     // Convert from Eigen object to ROS2 msg
            
            RL_pose_to_ROS(_feedback->desired.pose, desiredPose);                                   // Convert to ROS feedback msg from RobotLibrary pose
            
            Eigen_twist_to_ROS(_feedback->desired.twist, Eigen::VectorXd::Zero(6));                 // Convert to ROS2 feedback msg from Eigen::Vector            
            
            _feedback->manipulability = _controller->manipulability();                              // Proximity to a singularity
            
            _feedback->header.stamp = _node->now();                                                 // Time of publication
                  
            goalHandle->publish_feedback(_feedback);                                                // Make feedback available
            
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
            cleanup_and_send_result(3, "Failed to solve joint control.", goalHandle);               // Couldn't solve control; abort.
            return;
        }
    }
    
    // Technically, this should never be called.
    cleanup_and_send_result(1, "Follow transform completed.", goalHandle);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Completes the action and sends the result to the client                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
FollowTransform::cleanup_and_send_result(const int &status,
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
            RCLCPP_INFO(_node->get_logger(), "Follow transform finished. Awaiting new request.");
            break;
        }
        case 2:                                                                                     // Cancelled
        {
            goalHandle->canceled(result);
            RCLCPP_INFO(_node->get_logger(), "Follow transform cancelled. Awaiting new request.");
            break;
        }
        case 3:                                                                                     // Aborted
        {
            goalHandle->abort(result);
            RCLCPP_ERROR(_node->get_logger(), "Follow transform aborted.");
            break;
        }
    }

    _mutex->unlock();                                                                               // Release control
}

}
