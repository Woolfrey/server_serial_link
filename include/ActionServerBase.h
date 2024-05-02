#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ libraries
#include <rclcpp_action/rclcpp_action.hpp>                                                          // ROS2 action libaries
#include <serial_link_interfaces/action/move_to_joint_positions.hpp>                                // Custom action built in another project

// Short definitions for actions
using MoveToJointPositions = serial_link_interfaces::action::MoveToJointPositions;

// Short definitions for goal handler
using JointPositionsManager = rclcpp_action::ServerGoalHandle<MoveToJointPositions>;

template <class ControlType>
class ActionServerBase : public rclcpp::Node
{
     public:
     
          /**
           * Constructor.
           * @param options I don't really know what this does ¯\_(ツ)_/¯
           */
          ActionServerBase(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
          : Node("serial_link_action_server", options)
          {
               using namespace std::placeholders;
               
              // Create & advertise the action for moving to multiple joint positions
              // this->_jointPositionsServer
              // = rclcpp_action::create_server<MoveToJointPositions>(this,
              //                                                     "move_to_joint_positions",
              //                                                     std::bind(&ActionServerBase::move_to_joint_positions))
          }
          
     private:
     
          unsigned int _numberOfJoints;
     
          ControlType _controller;                                                                  // Template parameter for a RobotLibrary serial link controller
     
          rclcpp_action::Server<MoveToJointPositions>::SharedPtr _jointPositionsServer;             // Action server for handling MoveToJointPositions request
          
          /**
           * Processes action request to move to multiple joint positions.
           * @param uuid I don't know what this does 乁( ͡° ͜ʖ ͡°)ㄏ
           * @param request The Goal component of the MoveToJointPositions action
           * @return REJECT if request is not sound, otherwise ACCEPT_AND_EXECUTE
           */
          rclcpp_action::GoalResponse
          move_to_joint_positions(const rclcpp_action::GoalUUID &uuid,
                                  std::shared_ptr<const MoveToJointPositions::Goal> &request);        
          
          /**
           * Stop the robot from moving.
           * This is a VIRTUAL function and must be declared in the derived class.
           * @param requestManager Not used, but I don't think it compiles without it? ¯\_(⊙︿⊙)_/¯
           * @return ACCEPT (so far)
           */
          static rclcpp_action::CancelResponse
          stop(const std::shared_ptr<JointPositionsManager> requestManager);
          
          /**
           * Solve the feedback control to follow a joint trajectory.
           */
          static void
          track_joint_trajectory(const std::shared_ptr<JointPositionsManager> requestManager);
          
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Move the robot through multiple joint configurations                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class ControlType>
rclcpp_action::GoalResponse
ActionServerBase<ControlType>::move_to_joint_positions(const rclcpp_action::GoalUUID &uuid,
                                                       std::shared_ptr<const MoveToJointPositions::Goal> &request)
{
     (void)uuid;                                                                                    // This stops colcon from throwing a warning
    
     // Ensure the dimensions of inputs are sound
     if(request->waypoints.size() < 2)
     {
          std::string errorMessage = "A minimum number of 2 waypoints is required to generate a "
                                     "trajectory, but received only " + std::to_string(request->waypoints.size()) + ".";
     }
     else if(request->times.size() != request->waypoints.size())
     {
          std::string errorMessage = "Number of waypoints does not equal number of times ("
                                   + std::to_string(request->waypoints.size()) + " =/= "
                                   + std::to_string(request->times.size());
                                   
          RCLCPP_ERROR(this->get_logger(), errorMessage.c_str());                                   // Convert to char and publish
          
          return rclcpp_action::GoalResponse::REJECT;
     }
     else if(this->_controller->model->number_of_joints() != request->tracking_tolerance.size()
          or this->_controller->model->number_of_joints() != request->final_tolerance.size())
     {
          std::string errorMessage = "This robot has "
                                   + std::to_string(this->_controller->model->number_of_joints()) + 
                                    " but the tracking tolerance array had " + request->tracking_tolerance.size() +
                                    " elements, and the final tolerance array had "
                                   + request->final_tolerance.size() + " elements.";
          
          RCLCPP_ERROR(this->get_logger(), errorMessage.c_str());
          
          return rclcpp_action::GoalResponse::REJECT;
     }
     
     // 
     
     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
} 


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Solve the feedback control to track a joint trajectory                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void track_joint_trajectory(const std::shared_ptr<JointPositionsManager> requestManager)
{
     (void)requestManager;
     
     // These insane declarations are a consequence of the new ROS coding paradigm ಠ_ಠ
     MoveToJointPositions::Feedback::SharedPtr feedback
     = std::make_shared<MoveToJointPositions::Feedback>();                                          // This is contains information on tracking performance for the user
     
     MoveToJointPositions::Result::SharedPtr result
     = std::make_shared<MoveToJointPositions::Result>();                                            
     
     rclcpp::Rate loopRate(1);
     
     unsigned int counter = 1;
     
     while(rclcpp::ok())
     {
          if(counter == 1)
          {
               std::cout << "Worker bees can leave.\n";
               counter++;
          }
          else if(counter == 2)
          {
               std::cout << "Even drones can fly away.\n";
               counter++;
          }
          else
          {
               std::cout << "The Queen is their slave.\n";
               counter = 1;
          }
          
          loopRate.sleep();
    }
}
