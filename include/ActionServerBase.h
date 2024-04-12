#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ libraries
#include <rclcpp_action/rclcpp_action.hpp>                                                          // ROS2 action libaries
#include <serial_link_interfaces/action/movetojointposition.hpp>                                    // Custom action built in another project
#include <serial_link_interfaces/action/movetojointpositions.hpp>                                   // Custom action built in another project

// Short definitions for actions
using JointPositionAction      = serial_link_interfaces::action::MoveToJointPosition;
using MultiJointPositionAction = serial_link_interfaces::action::MoveToJointPositions;

// Short definitions for goal handler
using JointPositionManager      = rclcpp_action::ServerGoalHandle<JointPositionAction>;
using MultiJointPositionManager = rclcpp_action::ServerGoalHandle<MultiJointPositionAction>;

template <class DataType>
class ActionServerBase : public rclcpp::Node
{
     public:
     
          /**
           * Constructor.
           */
          ActionServerBase(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
               : Node("node_name", options)
          {
               RCLCPP::INFO(this->logger(), "Worker bees can leave.\n" +
                                            "Even drones can fly away.\n" +
                                            "The Queen is their slave.");
          }
          
     private:
     
          /**
           * Move the robot to a desired joint configuration. This is a goal request callback function.
           * @param uuid I have no idea what this does ¯\_(ツ)_/¯
           * @param request An instance of the MoveToJointPosition action Goal message
           * @return GoalResponse::REJECT = 1, GoalResponse::ACCEPT_AND_EXECUTE = 2, GoalResponse::ACCEPT_AND_DEFER = 3
           */
          rclcpp_action::GoalResponse move_to_position(const rclcpp_action::GoalUUID &uuid,
                                                       std::shared_ptr<const JointPositionAction::Goal> request)
          {
               (void)uuid;                                                                          // Stops colcon from throwing a warning
               
               // 1. Convert float64[] array in goal request to PositionVector msg type
               // 2. Create new goal server
               // 3. Call MoveToJointPositions action (plural)
          }
          
          /**
           * Move the robot through multiple joint configurations. This is a goal request callback function.
           * @param uuid I have no idea what this does ¯\_(ツ)_/¯
           * @param request An instance of the MoveToJointPosition action Goal message
           * @return GoalResponse::REJECT = 1, GoalResponse::ACCEPT_AND_EXECUTE = 2, GoalResponse::ACCEPT_AND_DEFER = 3
           */
          rclcpp_action::GoalResponse move_to_positions(const rclcpp_action::GoalUUID &uuid,
                                                        std::shared_ptr<const MultiJointPositionAction::Goal> request)
          {
               (void)uuid;                                                                          // Stops colcon from throwing a warning message
               
               // 1. Check request is sound
               // 2. Read joint state, update controller
               // 3. Create array of waypoints (first waypoint == current position) 
               // 5. Call track_joint_trajectory()
          }
          
          /**
           * Continuously solve feedback control to track a joint space trajectory. This is goal execution function.
           */
          void track_joint_trajectory()
          {
               // 1. Query desired state from trajectory, given elapsed time
               // 2. Solve feedback control
               // 3. Read joint state, update controller
          }
          
          /**
           * Stop the robot moving. This is a goal cancel callback function.
           * This is a virtual function and must be defined in a child class.
           * @return CancelResponse::REJECT = 1, CancelResponse::ACCEPT = 2
           */
          virtual rclcpp_action::CancelResponse stop() = 0;
          
          /**
           * Read joint values from the robot, update state of the underlying model.
           * @param jointState A data structure containing position, velocity, and effort (force/torque)
           */
          void read_joint_values(const sensor_msgs::msg::JointState &jointState)
          {
               // 
          }
};                                                                                                  // Semicolon needed after class declaration
