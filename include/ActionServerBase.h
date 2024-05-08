#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ libraries
#include <rclcpp_action/rclcpp_action.hpp>                                                          // ROS2 action libaries
#include <serial_link_interfaces/action/move_to_joint_states.hpp>                                   // Custom action built in another project
//#include <Trajectory/Spline.h>                                                                      // Use this to generate trajectories

// Short definitions for actions
using MoveToJointStates = serial_link_interfaces::action::MoveToJointStates;

// Short definitions for goal handler
using MoveToJointStatesManager = rclcpp_action::ServerGoalHandle<MoveToJointStates>;

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
              // = rclcpp_action::create_server<MoveToJointStates>(this,
              //                                                     "move_to_joint_positions",
              //                                                     std::bind(&ActionServerBase::move_to_joint_positions))
          }
          
     private:
     
          unsigned int _numberOfJoints;
     
          ControlType _controller;                                                                  // Template parameter for a RobotLibrary serial link controller
     
          rclcpp_action::Server<MoveToJointStates>::SharedPtr _jointPositionsServer;                // Action server for handling MoveToJointStates request
          
          /**
           * Processes action request to move to multiple joint positions.
           * @param uuid I don't know what this does 乁( ͡° ͜ʖ ͡°)ㄏ
           * @param request The Goal component of the MoveToJointStates action
           * @return REJECT if request is not sound, otherwise ACCEPT_AND_EXECUTE
           */
          rclcpp_action::GoalResponse
          move_to_joint_positions(const rclcpp_action::GoalUUID &uuid,
                                  std::shared_ptr<const MoveToJointStates::Goal> &request);        
          
          /**
           * Stop the robot from moving.
           * This is a VIRTUAL function and must be declared in the derived class.
           * @param requestManager Not used, but I don't think it compiles without it? ¯\_(⊙︿⊙)_/¯
           * @return ACCEPT (so far)
           */
          static rclcpp_action::CancelResponse
          stop(const std::shared_ptr<MoveToJointStatesManager> requestManager);
          
          /**
           * Solve the feedback control to follow a joint trajectory.
           */
          static void
          track_joint_trajectory(const std::shared_ptr<MoveToJointStatesManager> requestManager);
          
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Move the robot through multiple joint configurations                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class ControlType>
rclcpp_action::GoalResponse
ActionServerBase<ControlType>::move_to_joint_positions(const rclcpp_action::GoalUUID &uuid,
                                                       std::shared_ptr<const MoveToJointStates::Goal> &request)
{
     (void)uuid;                                                                                    // This stops colcon from throwing a warning
    
     // Check that the tolerances match the number of joints in the model
     unsigned int numberOfJoints = this->_controller->model.number_of_joints();                     // Simplifies things a little
     
     if(request->position_tracking_tolerance.size() != numberOfJoints
     or request->end_position_tolerance.size() != numberOfJoints)
     {
          std::string errorMessage = "This model has " + std::to_string(numberOfJoints) + " joints "
                                     "but the position tracking tolerance array had "
                                     + std::to_string(request->position_tracking_tolerance.size()) + 
                                     " elements, and the end position tolerance array had "
                                     + std::to_string(request->end_position_tolerance.size()) + " elements.";
          
          return rclcpp_action::GoalResponse::REJECT;
     }
     
     // Check that each waypoint is sound
     double previousTime;                                                                           // Used to check timing is correct
     unsigned int waypointNumber = 1;                                                               // For keeping track
     
     for(auto waypoint : request->waypoints)
     {
          // Check that dimensions match number of joints in robot model
          if(waypoint.position.size() != numberOfJoints
          or(waypoint.velocity.size() > 0     and waypoint.velocity.size()     != numberOfJoints)
          or(waypoint.acceleration.size() > 0 and waypoint.acceleration.size() != numberOfJoints))
          {
               std::string errorMessage = "This model has " + std::to_string(numberOfJoints) + " joints "
                                          "but the position array for waypoint " + std::to_string(waypointNumber) + " "
                                          "had " + std::to_string(waypoint.position.size()) + " elements, "
                                          "the velocity array had " + std::to_string(waypoint.velocity.size()) + " elements, and "
                                          "the acceleration array had " + std::to_string(waypoint.acceleration.size()) + " elements.";
                                          
               return rclcpp_action::GoalResponse::REJECT;
          }                 
          
          // Check that the times are in ascending order
          if(waypointNumber == 1 and waypoint.time < 0)
          {
               std::string errorMessage = "Time for the first waypoint was " + std::to_string(waypoint.time) + ", "
                                          "but cannot be negative.";
                                          
               return rclcpp_action::GoalResponse::REJECT;
          }
          else if(waypointNumber > 1 and waypoint.time <= previousTime)
          {
               std::string errorMessage  = "Time for waypoint " + std::to_string(waypointNumber) + " "
                                           "was less than the previous time (" + std::to_string(waypoint.time) +
                                           " > " + std::to_string(previousTime) + ").";
               
               return rclcpp_action::GoalResponse::REJECT;
          } 
          
          previousTime = waypoint.time;                                                             // Save for next loop
     }
     
     
     
     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;                                        // Call track_joint_trajectory()
} 


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Solve the feedback control to track a joint trajectory                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void track_joint_trajectory(const std::shared_ptr<MoveToJointStatesManager> requestManager)
{
     (void)requestManager;
     
     // These insane declarations are a consequence of the new ROS2 coding paradigm ಠ_ಠ
     MoveToJointStates::Feedback::SharedPtr feedback
     = std::make_shared<MoveToJointStates::Feedback>();                                             // This is contains real-time performance data for the user
     
     MoveToJointStates::Result::SharedPtr result
     = std::make_shared<MoveToJointStates::Result>();                                               // This contains a performance summary at the end
     
     rclcpp::Rate loopRate(1);                                                                      // Sets the frequency of the control thread
     
     
     // ↓↓↓ This is just for testing ↓↓↓
     
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
