/**
 * @file    hold_pose.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.0
 * @brief   Source files for the HoldPose class.
 *
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the control class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include <serial_link_action_server/hold_pose.hpp>

namespace serial_link_action_server {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
HoldPose::HoldPose(std::shared_ptr<rclcpp::Node> node,
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
    _feedback->header.frame_id = _controller->model()->base_name();                                 // Save this
    
    // Create publishers for RViz
    _posePublisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("pose_marker", 1);
    
    // Set static properties for arrows in RViz
    _arrowMarker.action          = visualization_msgs::msg::Marker::ADD;
    _arrowMarker.color.a         = _node->declare_parameter<double>("hold_pose.pose.alpha", 1.0);
    _arrowMarker.header.frame_id = _controller->model()->base_name();
    _arrowMarker.ns              = "hold_pose";
    _arrowMarker.scale.x         = _node->declare_parameter<double>("hold_pose.pose.scale", 0.015);
    _arrowMarker.scale.y         = _node->declare_parameter<double>("hold_pose.pose.shaft_diameter", 0.005);
    _arrowMarker.scale.z         = _node->declare_parameter<double>("hold_pose.pose.head_diameter", 0.001);
    _arrowMarker.type            = visualization_msgs::msg::Marker::ARROW;
}

  //////////////////////////////////////////////////////////////////////////////////////////////////// 
 //                          Process request to track Cartesian trajectory                         // 
////////////////////////////////////////////////////////////////////////////////////////////////////  
rclcpp_action::GoalResponse
HoldPose::handle_goal(const rclcpp_action::GoalUUID &uuid,
                      std::shared_ptr<const Action::Goal> goal)
{
    (void)uuid;                                                                                     // Prevents colcon from throwing a warning
       
    // Ensure arguments are sound
    if(goal->position_tolerance    <= 0.0
    or goal->orientation_tolerance <= 0.0)
    {
        RCLCPP_WARN(_node->get_logger(),
                    "Tolerances were not positive. Position tolerance was %f. Orientation tolerance was %f.",
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
    
    // Get desired pose
    
    _desiredPose = _controller->endpoint_pose();    

    // Generate pose marker
    
    _poseMarker.markers.clear();                                                                    // (Re)set
    

    size_t base_id = 3;                                                                             // Ensures unique IDs for each pose (3 arrows per pose)

    for (int axis = 0; axis < 3; ++axis)
    {
        visualization_msgs::msg::Marker arrow = _arrowMarker;                                       // Transfer static propertiess
        arrow.header.stamp = _node->now();                                                          // Stamp it with current time
        arrow.id           = base_id + axis;                                                        // Each marker needs a unique ID
        arrow.lifetime     = rclcpp::Duration::from_seconds(0.0);                                   // Length of time to visualise for
        
        Eigen::Matrix3d rotationMatrix = _desiredPose.quaternion().matrix();                        // Convert to 3x3 matrix               
        
        switch (axis)
        {
            case 0: arrow.color.r = 1.0; arrow.color.g = 0.0; arrow.color.b = 0.0; break;
            case 1: arrow.color.r = 0.0; arrow.color.g = 1.0; arrow.color.b = 0.0; break;
            case 2: arrow.color.r = 0.0; arrow.color.g = 0.0; arrow.color.b = 1.0; break;
        }

        geometry_msgs::msg::Point start, end;
        
        start.x = _desiredPose.translation().x();
        start.y = _desiredPose.translation().y();
        start.z = _desiredPose.translation().z();
        
        arrow.points.push_back(start);

        double length = 0.10;
        
        end.x = start.x + rotationMatrix.col(axis).x() * length;
        end.y = start.y + rotationMatrix.col(axis).y() * length;
        end.z = start.z + rotationMatrix.col(axis).z() * length;
         
        arrow.points.push_back(end);

        _poseMarker.markers.push_back(arrow);
    }

    _posePublisher->publish(_poseMarker);                                                          // Visualise markers
    
    // Make sure no other action is using the robot
    if(not _mutex->try_lock())
    {
        RCLCPP_WARN(_node->get_logger(),
                    "Request for hold pose action rejected. "
                    "Another action is currently using the robot.");
        
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;                                         // Return success and continue to execution
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       MAIN CONTROL LOOP                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////  
void
HoldPose::execute(const std::shared_ptr<GoalHandle> goalHandle)
{
    RCLCPP_INFO(_node->get_logger(), "Holding desired endpoint pose.");                             // Inform user
    
    auto goal = goalHandle->get_goal();                                                             // Save it so we can reference it later
    
    rclcpp::Rate loopRate(_controller->frequency());                                                // Used to regulate control loop timing
    
    unsigned long long int n = 1;                                                                   // Used for statistics    
    
    
    _desiredPose = _controller->endpoint_pose(); // FIX THIS LATER
    
    while(rclcpp::ok())
    {
        // Check to see if the action has been cancelled
        if (goalHandle->is_canceling())
        {
            cleanup_and_send_result(2, "Hold pose action cancelled.", goalHandle);
            
            return;
        }
        
        _controller->update();                                                                      // COmpute new Jacobian
               
        // Controller may throw a runtime error, so we need to catch it
        try
        {
            Eigen::VectorXd jointCommands = _controller->track_endpoint_trajectory(_desiredPose,
                                                                                   Eigen::Vector<double,6>::Zero(),
                                                                                   Eigen::Vector<double,6>::Zero());
            
            publish_joint_command(jointCommands);                                                   // Send immediately to robot
            
            // Update feedback fields
            RobotLibrary::Model::Pose actualPose = _controller->endpoint_pose();                    // Get computed pose
            RL_pose_to_ROS(_feedback->actual.pose, actualPose);                                     // Convert from RobotLibrary object to ROS2 msg
            
            Eigen::Vector<double,6> twist = _controller->endpoint_velocity();                       // Get computed endpoint velocity
            Eigen_twist_to_ROS(_feedback->actual.twist, twist);                                     // Convert from Eigen object to ROS2 msg
            
            RL_pose_to_ROS(_feedback->desired.pose, _desiredPose);                                  // Transfer desired pose to feedback message

            _feedback->manipulability = _controller->manipulability();                              // Proximity to singularity
            
            _feedback->header.stamp = _node->now();                                                 // Add a time stamp
                  
            goalHandle->publish_feedback(_feedback);                                                // Publish the feedback
            
            // Update error statistics for the result message
            Eigen::Vector<double,6> error = actualPose.error(_desiredPose); 
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

    cleanup_and_send_result(1, "This part of the code should never be called. How did that happen??.", goalHandle);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Completes the action and sends the result to the client                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
HoldPose::cleanup_and_send_result(const int &status,
                                                  const std::string &message,
                                                  const std::shared_ptr<GoalHandle> goalHandle)
{
    publish_joint_command(Eigen::VectorXd::Zero(_numJoints));                                       // Ensure the last command is zero
            
    // Assign data to the result section of the actions
    auto result = std::make_shared<Action::Result>();                                               // Result portion of the message
    result->position_error = _positionError;
    result->orientation_error = _orientationError;
    result->message = message;
 
    // Delete any poses that are remaining
    for (auto &marker : _poseMarker.markers)
    {
        marker.action = visualization_msgs::msg::Marker::DELETE;
        marker.header.stamp = _node->now();                                                         // always stamp with now
    }
    _posePublisher->publish(_poseMarker);
       
    switch(status)
    {
        case 1:                                                                                     // Successfully completed
        {
            goalHandle->succeed(result);
            RCLCPP_INFO(_node->get_logger(), "This should never be called. How did that happen?");
            break;
        }
        case 2:                                                                                     // Cancelled
        {
            goalHandle->canceled(result);
            RCLCPP_INFO(_node->get_logger(), "Hold pose action cancelled.");
            break;
        }
        case 3:                                                                                     // Aborted
        {    
            goalHandle->abort(result);
            RCLCPP_ERROR(_node->get_logger(), "Hold pose action aborted.");
            break;
        }
    }

    _mutex->unlock();                                                                               // Release control
}

}
