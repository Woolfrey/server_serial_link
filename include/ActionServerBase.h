#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ libraries
#include <rclcpp_action/rclcpp_action.hpp>                                                          // ROS2 action libaries
#include <serial_link_interfaces/action/move_to_joint_states.hpp>                                   // Custom action built in another project
#include <Trajectory/SplineTrajectory.h>

// Short definitions for actions
using MoveToJointStates = serial_link_interfaces::action::MoveToJointStates;

// Short definitions for goal handler
using MoveToJointStatesManager = rclcpp_action::ServerGoalHandle<MoveToJointStates>;

/**
 * This is a base class that defines common methods for all action servers for serial link robots.
 */
template <class ControlType>
class ActionServerBase : public rclcpp::Node
{
    public:
     
        /**
         * Constructor.
         * @param options I don't really know what this does ¯\_(ツ)_/¯
         */
        ActionServerBase(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
          
    private:
     
        unsigned int _numberOfJoints;                                                               ///< Actuated joints in the robot model.

        ControlType _controller;                                                                    ///< Template parameter for a RobotLibrary serial link controller

        rclcpp_action::Server<MoveToJointStates>::SharedPtr _jointPositionsServer;                  ///< Action server for handling MoveToJointStates request

        /**
        * Process request to move the robot through multiple joint states at specific times.
        * @param uuid I don't know what this does 乁( ͡° ͜ʖ ͡°)ㄏ
        * @param request The Goal component of the MoveToJointStates action
        * @return REJECT if request is not sound, otherwise ACCEPT_AND_EXECUTE
        */
        rclcpp_action::GoalResponse
        move_to_joint_states(const rclcpp_action::GoalUUID &uuid,
                             std::shared_ptr<const MoveToJointStates::Goal> request,
                             std::shared_ptr<const MoveToJointStates::Result> result);        

        /**
        * Stop the robot from moving.
        * This is a VIRTUAL function and must be declared in the derived class.
        * @param requestManager Not used, but I don't think it compiles without it? ¯\_(⊙︿⊙)_/¯
        * @return ACCEPT (so far)
        */
        rclcpp_action::CancelResponse
        stop(const std::shared_ptr<MoveToJointStatesManager> requestManager);

        /**
        * Solve the feedback control to follow a joint trajectory.
        */
        void
        track_joint_trajectory(std::shared_ptr<const MoveToJointStates::Goal> &request,
                               std::shared_ptr<const MoveToJointStates::Result> &result,
                               std::shared_ptr<const MoveToJointStates::Feedback> &feedback);
          
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class ControlType>
ActionServerBase<ControlType>::ActionServerBase(const rclcpp::NodeOptions &options)
                                                : Node("serial_link_action_server",options)
{
    using namespace std::placeholders;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Move the robot through multiple joint states                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class ControlType>
rclcpp_action::GoalResponse
ActionServerBase<ControlType>::move_to_joint_states(const rclcpp_action::GoalUUID &uuid,
                                                    std::shared_ptr<const MoveToJointStates::Goal> request,
                                                    std::shared_ptr<const MoveToJointStates::Result> result)
{
    unsigned int x = request->position_tracking_tolerance.size();
    unsigned int y = request->end_position_tolerance.size();
    
    RCLCPP_INFO(rclcpp::get_logger("serial_link_action_server"),
                "Received request for joint control.");
                
    // Check that at least 1 waypoint is provided
    if(request->states.size() == 0)
    {
        result->successful = -1; // INVALID_ARGUMENTS
        
        result->message = "At least 1 trajectory point must be specified, but you provided 0.";
        
        RCLCPP_INFO(rclcpp::get_logger("serial_link_action_server"),
                    "Request rejected. Insufficient waypoints provided.");
        
        return rclcpp_action::GoalResponse::REJECT;
    }
    // Check that the tolerances provided match the number of joints
    else if(x != this->_numberOfJoints or y != this->_numberOfJoints)
    {
        result->successful = -1; // INVALID_ARGUMENTS
        
        result->message = "Dimensions of arguments do not match. "
                          "This controller has " + std::to_string(this->_numberOfJoints) + " joints "
                          "but the position tracking tolerance array had " + std::to_string(x) + " elements, "
                          "and the end position tolerance array had " + std::to_string(y) + " elements.";
        
        RCLCPP_INFO(rclcpp::get_logger("serial_link_action_server"),
                    "Request rejected. Argument dimensions do not match the number of joints in the robot model.");
                            
        return rclcpp_action::GoalResponse::REJECT;
    }
    else
    {
        // Check that all the waypoints have the correct dimensions
        for(int i = 0; i < request->states.size(); i++)
        {
            unsigned int numPos = request->states[i].position.size();
            unsigned int numVel = request->states[i].velocity.size();
            unsigned int numAcc = request->states[i].acceleration.size();
            
            if(numPos != this->_numberOfJoints
            or numVel != this->_numberOfJoints
            or numAcc != this->_numberOfJoints)
            {
                result->successful = -1; // INVALID_ARGUMENTS
                
                result->message = "Incorrect dimensions specified for the joint states. "
                "This robot model has " + std::to_string(this->_numberOfJoints) + " joints, but "
                "the position vector for point " + std::to_string(i) + " had " + std::to_string(numPos) + " elements, "
                "the velocity vector had " + std::to_string(numVel) + " elements, and "
                "the acceleration vector had " + std::to_string(numAcc) + " elements.";
               
                RCLCPP_INFO(rclcpp::get_logger("serial_link_action_server"),
                            "Request rejected. State dimensions do not match the number of joints in the robot model.");                   
                
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
                    
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;                                     // Go immediately to `track_joint_trajectory()`
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Solve the feedback control to track a joint trajectory                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class ControlType>
void
ActionServerBase<ControlType>::track_joint_trajectory(std::shared_ptr<const MoveToJointStates::Goal> &request,
                                                      std::shared_ptr<const MoveToJointStates::Result> &result,
                                                      std::shared_ptr<const MoveToJointStates::Feedback> &feedback)
{

    RCLCPP_INFO(rclcpp::get_logger("serial_link_action_server"),
                "Request accepted. Running joint control mode.");
    try
    {
        // Set up the feedback
        serial_link_interfaces::msg::Statistics statistics[this->_numberOfJoints];

        // Create the trajectory object
        std::vector<double> times;
        times.push_back(request->delay);                                                            // Use the delay specified
        
        std::vector<Eigen::VectorXd> positions;
        positions.push_back(this->_controller->model.joint_positions());                            // Use the current state
        
        for(auto state : request->states)
        {
            times.push_back(state.time);
            positions.push_back(Eigen::VectorXd(state.position.data()));
        }
        
        SplineTrajectory<double> trajectory(positions, times, this->_controller->model.joint_velocities());

        // Run the control loop
        rclcpp::Rate loopRate(this->_controller.frequency());
        
        double startTime = this->get_clock()->now().seconds();
        
        while(rclcpp::ok())
        {
            double elapsedTime = this->get_clock()->now().seconds() - startTime;
        }
    }
    catch(const std::exception &exception)
    {
        result->successful = -4; // OTHER    
        result->message = exception.what();
    }
}
