/**
 * @file    TrackCartesianTrajectory.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   A ROS2 action that enables the endpoint of a robot arm to follow a trajectory.
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

#include <FollowTwist.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
FollowTwist::FollowTwist(std::shared_ptr<rclcpp::Node> node,
                         std::shared_ptr<RobotLibrary::Control::SerialLinkBase> controller,
                         std::shared_ptr<std::mutex> mutex,
                         const std::string &actionName,
                         const std::string &controlTopicName,
                         const std::string &twistTopicName)
                         : ActionServerBase(node,
                                            controller,
                                            mutex,
                                            actionName,
                                            controlTopicName)
{
    _feedback->header.frame_id = _controller->model()->base_name();                                 // Save this
     
    _twistSubscriber = node->create_subscription<geometry_msgs::msg::TwistStamped>
    (
        twistTopicName,
        1,
        std::bind(&FollowTwist::twist_callback, this, std::placeholders::_1)
    );
}

  //////////////////////////////////////////////////////////////////////////////////////////////////// 
 //                          Process request to track Cartesian trajectory                         // 
////////////////////////////////////////////////////////////////////////////////////////////////////  
rclcpp_action::GoalResponse
FollowTwist::handle_goal(const rclcpp_action::GoalUUID &uuid,
                         std::shared_ptr<const Action::Goal> goal)
{
    (void)uuid;                                                                                     // Prevents colcon from throwing a warning
    
    // Ensure arguments are sound
    if(goal->linear_tolerance <= 0.0
    or goal->angular_tolerance <= 0.0)
    {
        RCLCPP_WARN(_node->get_logger(),
                    "Velocity error tolerances were not positive. "
                    "Linear tolerance was %f. "
                    "Angular tolerance was %f.",
                    goal->linear_tolerance,
                    goal->angular_tolerance);

        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Check that the subscription topic exists
    if (_node->count_publishers(_twistSubscriber->get_topic_name()) == 0)
    {
        RCLCPP_WARN(_node->get_logger(), "Topic '%s' is not advertised. Rejecting goal.", _twistSubscriber->get_topic_name());
        
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // (Re)set statistics
    _linearError.mean = 0.0;
    _linearError.variance = 0.0;
    _linearError.min = std::numeric_limits<double>::max();
    _linearError.max = std::numeric_limits<double>::lowest();

    _angularError.mean = 0.0;
    _angularError.variance = 0.0;
    _angularError.min = std::numeric_limits<double>::max();
    _angularError.max = std::numeric_limits<double>::lowest();
   
    
    // Make sure no other action is using the robot
    if(not _mutex->try_lock())
    {
        RCLCPP_WARN(_node->get_logger(), "Request for Cartesian trajectory tracking rejected. "
                                         "Another action is currently using the robot.");
        
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;                                         // Return success and continue to execution
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       MAIN CONTROL LOOP                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////  
void
FollowTwist::execute(const std::shared_ptr<GoalHandle> goalHandle)
{
    RCLCPP_INFO(_node->get_logger(), "Executing resolved motion rate control.");                    // Inform user
    
    auto goal = goalHandle->get_goal();                                                             // Save it so we can reference it later
    
    rclcpp::Rate loopRate(_controller->frequency());                                                // Used to regulate control loop timing
    
    unsigned long long int n = 1;                                                                   // Used for statistics    
    
    while(rclcpp::ok())
    {
        // Check to see if the action has been cancelled
        if (goalHandle->is_canceling())
        {
            cleanup_and_send_result(2, "Resolved motion rate control cancelled.", goalHandle);
            return;
        }
        
        // Get the latest twist
        geometry_msgs::msg::TwistStamped input;
        
        std::lock_guard<std::mutex> lock(_twistMutex);                                              // Prevent subscriber from simultaneous access
        
        if(_lastTwist and (_node->now() - _lastTwist->header.stamp).seconds() < goal->timeout)
        {
            input = *_lastTwist;                                                                    // Transfer
        }
        else
        {
            cleanup_and_send_result(3, "Timeout: no new twist command received.", goalHandle);      // Abort
            return;
        }
        
        // Transfer desired value
        Eigen::Vector<double,6> desiredTwist;
        desiredTwist[0] = input.twist.linear.x;
        desiredTwist[1] = input.twist.linear.y;
        desiredTwist[2] = input.twist.linear.z;
        desiredTwist[3] = input.twist.angular.x;
        desiredTwist[4] = input.twist.angular.y;
        desiredTwist[5] = input.twist.angular.z;
        
        _controller->update();                                                                      // Compute new Jacobian
        
        // Controller could throw an error, so we need to catch it
        try
        {
            Eigen::VectorXd jointCommands = _controller->resolve_endpoint_twist(desiredTwist);      // Solve control
            
            publish_joint_command(jointCommands);                                                   // Send to robot immediately
            
            Eigen::Vector<double,6> actualTwist = _controller->endpoint_velocity();
            
            // Update feedback fields 
            Eigen_twist_to_ROS(_feedback->actual.twist, actualTwist);
            
            Eigen_twist_to_ROS(_feedback->desired.twist, desiredTwist);

            _feedback->manipulability = _controller->manipulability();
            
            _feedback->header.stamp = _node->now();
            
            goalHandle->publish_feedback(_feedback);
            
            // Update error statistics for result message
            Eigen::VectorXd twistError = desiredTwist - _controller->endpoint_velocity();
            double linearError = twistError.head(3).norm();
            double angularError = twistError.tail(3).norm();
            update_statistics(_linearError, linearError, n);
            update_statistics(_angularError, angularError, n);
            
            // Check tolerances
            if (linearError  > goal->linear_tolerance
            or  angularError > goal->angular_tolerance)
            {
                cleanup_and_send_result(3, "Velocity error tolerance violated.", goalHandle);       // Abort
                return;
            }
            
            ++n;                                                                                    // Increment sample size
            
            loopRate.sleep();                                                                       // Synchronise with control frequency                   
        }
        catch (const std::exception &exception)
        {
            RCLCPP_ERROR(_node->get_logger(), exception.what());
            cleanup_and_send_result(3, "Resolved motion rate control failed.", goalHandle);         // Couldn't solve control; abort.
            return;
        }
    }
    
    // Technically, this should never be called.
    cleanup_and_send_result(1, "Follow twist action completed.", goalHandle);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Store new twist commands                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
FollowTwist::twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(_twistMutex);                                                  // Prevents control thread from accessing data at same time
    _lastTwist = *msg;                                                                              // Save
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Completes the action and sends the result to the client                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
FollowTwist::cleanup_and_send_result(const int &status,
                                     const std::string &message,
                                     const std::shared_ptr<GoalHandle> goalHandle)
{
    publish_joint_command(Eigen::VectorXd::Zero(_numJoints));                                       // Ensure the last command is zero
            
    // Assign data to the result section of the actions
    auto result = std::make_shared<Action::Result>();                                               // Result portion of the message
    result->linear_error = _linearError;
    result->angular_error = _angularError;
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
