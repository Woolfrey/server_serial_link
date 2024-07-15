/**
 * @file   TrackJointTrajectory.h
 * @author Jon Woolfrey
 * @data   June 2024
 * @brief  A ROS2 action server for joint trajectory tracking.
 */

#ifndef TRACKJOINTTRAJECTORY_H_
#define TRACKJOINTTRAJECTORY_H_

#include <Eigen/Dense>                                                                              // Can use Eigen::Map?
#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ libraries
#include <rclcpp_action/rclcpp_action.hpp>                                                          // ROS2 Action C++ libraries
#include <serial_link_interfaces/action/track_joint_trajectory.hpp>                                 // Previously built package

#include <RobotLibrary/SplineTrajectory.h>

// Short naming conventions for easier referencing
using JointControlAction = serial_link_interfaces::action::TrackJointTrajectory;
using JointControlManager = rclcpp_action::ServerGoalHandle<JointControlAction>;

/**
 * This class performs joint trajectory tracking for a serial link robot arm.
 */
class TrackJointTrajectory : public rclcpp::Node
{
    public:
        
        /**
         * Constructor for the class.
         * @param options I have no idea what this does ¯\_(ツ)_/¯
         */
        TrackJointTrajectory(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
    private:

        unsigned int _numJoints = 3; // CHANGE THIS IN FUTURE
            
        rclcpp_action::Server<JointControlAction>::SharedPtr _actionServer;                         // This is the foundation for the class
        
        std::shared_ptr<JointControlAction::Feedback> _feedback 
        = std::make_shared<JointControlAction::Feedback>();                                         // Use this to store feedback
        
        std::vector<serial_link_interfaces::msg::Statistics> _errorStatistics;                      // Stored data on position tracking error
        
        SplineTrajectory _trajectory;                                                               // Trajectory object
        
        /**
         * Processes the request for the TrackJointTrajectory action.
         * @param uuid I have no idea what this does ¯\_(ツ)_/¯
         * @param request The goal component of the TrackJointControl action definition.
         * @return REJECT if the arguments are not sound, ACCEPT_AND_EXECUTE otherwise.
         */
        inline
        rclcpp_action::GoalResponse
        request_tracking(const rclcpp_action::GoalUUID &uuid,
                         std::shared_ptr<const JointControlAction::Goal> request);
        
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
TrackJointTrajectory::TrackJointTrajectory(const rclcpp::NodeOptions &options)
                                           : Node("joint_tracking_server", options)
{
    using namespace std::placeholders;

    this->_actionServer = rclcpp_action::create_server<JointControlAction>
    (this, "track_joint_trajectory",
     std::bind(&TrackJointTrajectory::request_tracking, this, _1, _2),
     std::bind(&TrackJointTrajectory::cancel, this, _1),
     std::bind(&TrackJointTrajectory::track_joint_trajectory,this,_1)
    );
 
    ///////////////// Set the size of arrays based on number of joints in robot model //////////////
    
    this->_feedback->actual.position.resize(_numJoints);
    this->_feedback->actual.velocity.resize(_numJoints);
//  this->_feedback->actual.acceleration.resize(_numJoints);                                        // Not available for actual joint state
    this->_feedback->actual.effort.resize(_numJoints);
    
    this->_feedback->desired.position.resize(_numJoints);
    this->_feedback->desired.velocity.resize(_numJoints);
    this->_feedback->desired.acceleration.resize(_numJoints);                                       // Not available for desired joint state
//  this->_feedback->desired.effort.resize(_numJoints);
    
    this->_feedback->error.position.resize(_numJoints);
    this->_feedback->error.velocity.resize(_numJoints);
//  this->_feedback->error.acceleration.resize(_numJoints);                                         // Not available for error
//  this->_feedback->error.effort.resize(_numJoints);                                               // Not available for error
    
    this->_errorStatistics.resize(_numJoints);                                                      // Data on position tracking error
                                                                         
    RCLCPP_INFO(this->get_logger(), "Server initiated. Awaiting action request.");
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Process a request to perform joint trajectory tracking                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
inline
rclcpp_action::GoalResponse
TrackJointTrajectory::request_tracking(const rclcpp_action::GoalUUID &uuid,
                                       std::shared_ptr<const JointControlAction::Goal> request)
{
    (void)uuid;
    
    RCLCPP_INFO(this->get_logger(), "Request for joint trajectory tracking received.");
    
    // Get the trajectory data from the request
    std::vector<double> times;
    std::vector<Eigen::VectorXd> positions;
    for(auto &point : request->points) 
    {
        if(point.position.size() != this->_numJoints)
        {
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        auto blah = point.position; // IMPROVE THIS?
        Eigen::Map<Eigen::VectorXd> vectorMap(blah.data(), blah.size());
        positions.emplace_back(vectorMap);
        
        times.push_back(point.time);
    }
    
    // Try to create the trajectory
    try
    {
        this->_trajectory = SplineTrajectory(positions, times,
                                             Eigen::VectorXd::Zero(this->_numJoints));
    }
    catch(const std::exception &exception)
    {
        std::cerr << exception.what();  // NOTE: Need to send this back as the result message
        
        return rclcpp_action::GoalResponse::REJECT;
    }

    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;                                         // This immediately calls `track_joint_trajectory()`
}
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Cancel a joint trajectory tracking action                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
inline
rclcpp_action::CancelResponse
TrackJointTrajectory::cancel(const std::shared_ptr<JointControlManager> actionManager)
{ 
    RCLCPP_INFO(this->get_logger(), "Received request to cancel joint trajectory tracking.");
    
    auto result = std::make_shared<JointControlAction::Result>();                                   // Result portion of the action          
    result->successful = -4;                                                                        // CANCELLED
    result->message = "Action cancelled.";                                                          // Info for the client
    
    actionManager->canceled(result);                                                                // Put the result in 

    return rclcpp_action::CancelResponse::ACCEPT;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       MAIN CONTROL LOOP                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////       
inline
void
TrackJointTrajectory::track_joint_trajectory(const std::shared_ptr<JointControlManager> actionManager)
{
    // These need to be removed / replaced in future:
    rclcpp::Rate loopRate(100);                                                                     // This regulates the control frequency

    // Variables used in this scope
    auto request  = actionManager->get_goal();                                                      // Retrieve goal
  
    // Set initial values for error
    for(auto &error : this->_errorStatistics)
    {
        error.mean     = 0.0;
        error.min      = 0.0;
        error.max      = 0.0;
        error.variance = 0.0;
    }
        
    // Variables used for timing
    rclcpp::Clock timer;                                                                            // Clock object
    double startTime = timer.now().seconds();                                                       // Start the timer
    double elapsedTime;                                                                             // Used in the control loop
    
    // Delay the start of the trajectory tracing
    if(request->delay > 0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Counting down...");
        
        while(rclcpp::ok())
        {
            this->_feedback->time_remaining = request->delay - (timer.now().seconds() - startTime); // As it says
            
            if(this->_feedback->time_remaining <= 0.0) break;                                       // I have to do this weird check because it wasn't breaking the loop correctly
            else
            {             
                RCLCPP_INFO(this->get_logger(), "%i", (int)this->_feedback->time_remaining);        // Round down to whole second
                rclcpp::sleep_for(std::chrono::seconds(1));
            }
        }
    }
    
    // Run the control
    RCLCPP_INFO(this->get_logger(), "Executing joint trajectory tracking.");
    startTime = timer.now().seconds();                                                              // (Re)start the timer
    do
    {   
        elapsedTime = timer.now().seconds() - startTime;                                            // Get the elapsed time since the start

        // Get the desired state from the trajectory generator
        const auto &[desiredPosition,
                     desiredVelocity,
                     desiredAcceleration] = this->_trajectory.query_state(elapsedTime);
        
        // Cycle through the joints
        for(unsigned int j = 0; j < this->_numJoints; j++)
        {
            this->_feedback->desired.position[j]     = desiredPosition[j];
            this->_feedback->desired.velocity[j]     = desiredVelocity[j];
            this->_feedback->desired.acceleration[j] = desiredAcceleration[j];
            
            this->_errorStatistics[j].mean = (this->_feedback->error.position[j]
                                           + (n-1)*this->_errorStatistics[j].mean)/n;
            
            
        }
        
        actionManager->publish_feedback(this->_feedback);
        
        loopRate.sleep();                                                                           // Synchronize
        
    }while(rclcpp::ok() and elapsedTime <= request->points.back().time); // NOTE: Using do/while can fail with the timer here?
    
    // Send the result to the client
    if(rclcpp::ok())
    {
//        actionManager->succeed(result);                                                             // Fill in the result message
        RCLCPP_INFO(this->get_logger(), "Trajectory tracking complete.");
    }
}
            
#endif
