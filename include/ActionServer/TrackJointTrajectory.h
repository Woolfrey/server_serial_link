/**
 * @file   TrackJointTrajectory.h
 * @author Jon Woolfrey
 * @data   June 2024
 * @brief  A ROS2 action server for joint trajectory tracking.
 */

#ifndef TRACKJOINTTRAJECTORY_H_
#define TRACKJOINTTRAJECTORY_H_

#include <serial_link_interfaces/action/track_joint_trajectory.hpp>                                 // Previously built package
#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ libraries
#include <rclcpp_action/rclcpp_action.hpp>                                                          // ROS2 Action C++ libraries

// Short naming conventions for easier referencing
using JointControlAction = serial_link_interfaces::action::TrackJointTrajectory;
using JointControlManager = rclcpp_action::ServerGoalHandle<JointControlAction>;

class TrackJointTrajectory : public rclcpp::Node
{
    public:
        
        /**
         * Constructor for the class.
         * @param options I have no idea what this does ¯\_(ツ)_/¯
         */
        TrackJointTrajectory(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
                             : Node("joint_trajectory_tracking_server", options)
        {
            using namespace std::placeholders;
            
            this->_actionServer = rclcpp_action::create_server<JointControlAction>
            (this,
             "track_joint_trajectory",
             std::bind(&TrackJointTrajectory::request_joint_control, this, _1, _2),
             std::bind(&TrackJointTrajectory::cancel, this, _1),
             std::bind(&TrackJointTrajectory::track_joint_trajectory,this,_1)
            );
                                                                                  
            RCLCPP_INFO(this->get_logger(), "Server initiated. Awaiting action request.");
        }
    
    private:
    
        rclcpp_action::Server<JointControlAction>::SharedPtr _actionServer;                         // This is the fundamental object
        
        /**
         * Processes the request for the TrackJointTrajectory action.
         * @param uuid I have no idea what this does ¯\_(ツ)_/¯
         * @param request The goal component of the TrackJointControl action definition.
         * @return REJECT if the arguments are not sound, ACCEPT_AND_EXECUTE otherwise.
         */
        inline
        rclcpp_action::GoalResponse
        request_joint_control(const rclcpp_action::GoalUUID &uuid,
                              std::shared_ptr<const JointControlAction::Goal> request)
        {
            (void)uuid;
            (void)request;
            
            RCLCPP_INFO(this->get_logger(), "Request for joint trajectory tracking received.");
            
            // NOTE TO SELF: Create the trajectory here? If it fails, reject.
            
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;                                // This immediately calls `track_joint_trajectory()`
        }
        
        /**
         * Handles the cancel request.
         */
        inline
        rclcpp_action::CancelResponse
        cancel(const std::shared_ptr<JointControlManager> actionManager)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel joint trajectory tracking.");
            
            (void)actionManager;                                                                    // This stops any warnings during compilation
            
            return rclcpp_action::CancelResponse::ACCEPT;
        }
        
        /**
         * This is the main control loop for joint trajectory tracking.
         */
        inline
        void
        track_joint_trajectory(const std::shared_ptr<JointControlManager> actionManager)
        {
            auto request = actionManager->get_goal();                                               // Retrieve goal
            auto result  = std::make_shared<JointControlAction::Result>();                          // Use this to store the result
            auto feedback = std::make_shared<JointControlAction::Feedback>();
            
            // Timing stuff
            rclcpp::Clock timer;                                                                    // Clock object
            double startTime = timer.now().seconds();
            double elapsedTime;                                                                     // Used in the control loop
            
            ////////////////////////////////////////////////////////////////////////////////////////
            if(request->delay > 0.0)
            {
                RCLCPP_INFO(this->get_logger(), "Delay requested. Counting down.");
                
                feedback->time_remaining = 1.0;                                                     // Arbitrary
                while(rclcpp::ok())
                {
                    feedback->time_remaining = request->delay - (timer.now().seconds() - startTime); // As it says
                    
                    // I have to do this weird check because it wasn't breaking the loop correctly
                    if(feedback->time_remaining <= 0.0) break;
                    else
                    {             
                        RCLCPP_INFO(this->get_logger(), "%i", (int)feedback->time_remaining);       // Round down to whole second
                        rclcpp::sleep_for(std::chrono::seconds(1));
                    }
                }
            }
                
            ////////////////////////////////////////////////////////////////////////////////////////
            
            rclcpp::Rate loopRate(0.5);                                                             // This regulates the control frequency

            unsigned int count = 1;
            
            RCLCPP_INFO(this->get_logger(), "Executing joint trajectory tracking.");
            
            do
            {   
                     elapsedTime = timer.now().seconds() - startTime;
                     
                     if(count == 1) 
                     {
                        RCLCPP_INFO(this->get_logger(), "Worker bees can leave.");
                        count++;
                     }
                    else if(count == 2)
                    {
                        RCLCPP_INFO(this->get_logger(), "Even drones can fly away.");
                        count++;
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "The Queen is their slave.");
                        count = 1;
                    }
                
                loopRate.sleep();                                                                   // Synchronize
                
            }while(rclcpp::ok() and elapsedTime <= request->points.back().time);
            
            if(rclcpp::ok())
            {
                actionManager->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Trajectory tracking complete.");
            }
        }    
};                                                                                                  // Semicolon required after a class declaration

#endif
