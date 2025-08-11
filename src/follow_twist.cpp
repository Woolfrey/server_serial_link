/**
 * @file    follow_twist.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.1
 * @brief   Source code for the FollowTwist action server.
 * 
 * @details This action enables a serial link robot arm to follow a Cartesian velocity command (twist)
 *          in real time. It subscribes to a specified geometry_msgs::msg::TwistStamped topic, and uses
 *          that twist to solve for the necessary joint control. It then publishes the joint control
 *          over the ROS2 network.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the control class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include <serial_link_action_server/follow_twist.hpp>

namespace serial_link_action_server {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
FollowTwist::FollowTwist(std::shared_ptr<rclcpp::Node> node,
                         std::shared_ptr<RobotLibrary::Control::SerialLinkBase> controller,
                         std::shared_ptr<std::mutex> mutex,
                         const std::string &actionName,
                         const std::string &controlTopicName)
                         
: ActionServerBase(node, controller, mutex, actionName, controlTopicName),
  _transformBuffer(node->get_clock()),                                                              // Pass the node's clock to Buffer
  _transformListener(_transformBuffer)                                                              // Attach listener to buffer
{
    _feedback->header.frame_id = _controller->model()->base_name();                                 // Save this
    
    _markerArrayPublisher = _node->create_publisher<visualization_msgs::msg::MarkerArray>("twist_markers",1);
    
    // Set static properties for marker(s)
    _defaultMarker.color.r         = 0.0;
    _defaultMarker.color.g         = 0.0;
    _defaultMarker.color.b         = 1.0;
    _defaultMarker.color.a         = 1.0;
    _defaultMarker.header.frame_id = _controller->model()->base_name();
    _defaultMarker.id              = 0;
    _defaultMarker.lifetime        = rclcpp::Duration::from_seconds(0.2);
    _defaultMarker.scale.x         = 0.02;
    _defaultMarker.scale.y         = 0.04;
    _defaultMarker.scale.z         = 0.06;
    _defaultMarker.type            = visualization_msgs::msg::Marker::ARROW;
}

  //////////////////////////////////////////////////////////////////////////////////////////////////// 
 //                                Process request to follow twist                                 // 
////////////////////////////////////////////////////////////////////////////////////////////////////  
rclcpp_action::GoalResponse
FollowTwist::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Action::Goal> goal)
{
    (void)uuid;                                                                                     // Prevents colcon from throwing a warning
    
    RCLCPP_INFO(_node->get_logger(), "Received request to follow twist.");                          // Inform the user
    
    // Ensure arguments are sound
    if(goal->linear_tolerance  <= 0.0
    or goal->angular_tolerance <= 0.0)
    {
        RCLCPP_WARN(_node->get_logger(),
                    "Velocity error tolerances were not positive. "
                    "Linear tolerance was %f. Angular tolerance was %f.",
                    goal->linear_tolerance, goal->angular_tolerance);

        return rclcpp_action::GoalResponse::REJECT;
    }
    else if(goal->timeout <= 0.0)
    {
        RCLCPP_WARN(_node->get_logger(),
                    "Timeout was %f seconds, but must be positive.",
                    goal->timeout);
        
        return rclcpp_action::GoalResponse::REJECT;
    }
    else if(_node->count_publishers(goal->topic_name) == 0)
    {
        RCLCPP_WARN(_node->get_logger(), "No publishers found on topic '%s'.", goal->topic_name.c_str());
        
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
    
    // Create the subscriber
    _twistSubscriber = _node->create_subscription<geometry_msgs::msg::TwistStamped>
    (
        goal->topic_name,
        1,
        std::bind(&FollowTwist::twist_callback, this, std::placeholders::_1)
    );
    
    _lastTwistHeader.stamp = _node->now();
    _lastTwistHeader.frame_id = "unset";
    
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
FollowTwist::execute(const std::shared_ptr<GoalHandle> goalHandle)
{
    auto goal = goalHandle->get_goal();                                                             // Save it so we can reference it later

    RCLCPP_INFO(_node->get_logger(), "Following twist commands on the `%s` topic.", goal->topic_name.c_str()); // Inform the user
      
    rclcpp::Rate loopRate(_controller->frequency());                                                // Used to regulate control loop timing
    
    unsigned long long int n = 1;                                                                   // Used for statistics    

    // We need to give the node a bit of time to execute the twist_callback() method
    while (rclcpp::ok() && _lastTwistHeader.frame_id == "unset") 
    {
        if ((_node->now() - _lastTwistHeader.stamp).seconds() > goal->timeout)
        {
            cleanup_and_send_result(3, "Timeout: Failed to receive the first twist message in "
                                    + std::to_string((_node->now() - _lastTwistHeader.stamp).seconds()), goalHandle);
            return;
        }
        
        rclcpp::sleep_for(std::chrono::milliseconds(10));                                           // Small delay to allow messages to be processed
    }
  
    while(rclcpp::ok())
    {
        // Check for
        if (goalHandle->is_canceling())
        {
            cleanup_and_send_result(2, "Follow twist cancelled.", goalHandle);
            return;
        }
        
        // Check for timeout
        double elapsedTime = (_node->now() - _lastTwistHeader.stamp).seconds();
        if (elapsedTime > goal->timeout)
        {
             cleanup_and_send_result(3, "Timeout: No new twist command received in " + std::to_string(elapsedTime) + " seconds.", goalHandle); // Abort
             return;
        }
        
        // Try and look up the transform
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = _transformBuffer.lookupTransform(_controller->model()->base_name(), _lastTwistHeader.frame_id, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &exception)
        {
            RCLCPP_ERROR(_node->get_logger(), "Failed to look up transform: %s", exception.what());
            cleanup_and_send_result(3, exception.what(), goalHandle);                               // Abort
            return;
        }
        
        // Rotate the twist to the correct frame
        geometry_msgs::msg::Vector3Stamped linearRotated, angularRotated;
        tf2::doTransform(_linearVelocity, linearRotated, transform); 
        tf2::doTransform(_angularVelocity, angularRotated, transform);           
   
        // Transfer desired value
        Eigen::Vector<double,6> desiredTwist{linearRotated.vector.x,
                                             linearRotated.vector.y,
                                             linearRotated.vector.z,
                                             angularRotated.vector.x,
                                             angularRotated.vector.y,
                                             angularRotated.vector.z};
        
        _controller->update();                                                                      // Compute new Jacobian
        
        // Controller could throw an error, so we need to catch it
        try
        {
            publish_joint_command(_controller->resolve_endpoint_twist(desiredTwist));               // Solve the control and send straight to robot                                                  // Send to robot immediately
            
            // Update & publish feedback
            
            Eigen::Vector<double,6> actualTwist = _controller->endpoint_velocity();
            
            Eigen::Vector<double,6> twistError = desiredTwist - actualTwist;
            
            _feedback->linear_error = twistError.head(3).norm();
            
            _feedback->angular_error = twistError.tail(3).norm();

            _feedback->manipulability = _controller->manipulability();
            
            _feedback->header.stamp = _node->now();
            
            goalHandle->publish_feedback(_feedback);
            
            // Update error statistics for result message
            update_statistics(_linearError, _feedback->linear_error, n);

            update_statistics(_angularError, _feedback->angular_error, n);
            
            ++n;                                                                                    // Increment sample size
            
            // Publish visualisation data
            Eigen::Vector3d endpointPosition = _controller->endpoint_pose().translation();
            
            visualization_msgs::msg::Marker desiredMarker = _defaultMarker;
            desiredMarker.action = visualization_msgs::msg::Marker::ADD;
            desiredMarker.header.stamp = _node->now();
            desiredMarker.id = 0;
 
            geometry_msgs::msg::Point start, end;
    
            start.x = endpointPosition.x();
            start.y = endpointPosition.y();
            start.z = endpointPosition.z();
            
            end.x = start.x + desiredTwist[0];
            end.y = start.y + desiredTwist[1];
            end.z = start.z + desiredTwist[2];

            desiredMarker.points.push_back(start);
            desiredMarker.points.push_back(end);
            
            visualization_msgs::msg::Marker actualMarker = desiredMarker;
            actualMarker.color.r = 0.0; actualMarker.color.g = 1.0; actualMarker.color.b = 0.0;
            actualMarker.id = 1;
            
            end.x = start.x + actualTwist[0];
            end.y = start.y + actualTwist[1];
            end.z = start.z + actualTwist[2];

            actualMarker.points.clear();
            actualMarker.points.push_back(start);
            actualMarker.points.push_back(end);          
            
            // Add to array
            _markerArray.markers.clear();
            _markerArray.markers.push_back(desiredMarker);
            _markerArray.markers.push_back(actualMarker);
            _markerArrayPublisher->publish(_markerArray);
            
            // Check tolerances
            if (_feedback->linear_error > goal->linear_tolerance)
            {
                cleanup_and_send_result(3, "Linear velocity error tolerance violated: "
                                        + std::to_string(_feedback->linear_error) + " >= "
                                        + std::to_string(goal->linear_tolerance),
                                        goalHandle);
                return;
            }
            else if (_feedback->angular_error > goal->angular_tolerance)
            {
                cleanup_and_send_result(3, "Angular velocity error tolerance violated: "
                                        + std::to_string(_feedback->angular_error) + " >= "
                                        + std::to_string(goal->angular_tolerance),
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
    
    cleanup_and_send_result(1, "Follow twist completed.", goalHandle);                              // Technically, this should never be called.
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Store new twist commands                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
FollowTwist::twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr input)
{
    // NOTE: We need to save values as Vector3 so we can rotate them later
    _linearVelocity.header = input->header;
    _linearVelocity.vector = input->twist.linear;
    
    _angularVelocity.header = input->header;
    _angularVelocity.vector = input->twist.angular;
    
    _lastTwistHeader = input->header;
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
      
    _twistSubscriber.reset();                                                                       // Stop subscriber to free up resources      
    
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
            RCLCPP_INFO(_node->get_logger(), "Follow twist finished. Awaiting new request.");
            break;
        }
        case 2:                                                                                     // Cancelled
        {
            goalHandle->canceled(result);
            RCLCPP_INFO(_node->get_logger(), "Follow twist cancelled. Awaiting new request.");
            break;
        }
        case 3:                                                                                     // Aborted
        {
            goalHandle->abort(result);
            RCLCPP_ERROR(_node->get_logger(), "Follow twist aborted.");
            break;
        }
    }

    _mutex->unlock();                                                                               // Release control
}

} // namespace
