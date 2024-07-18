/**
 * @file   JointStateSubscriber.h
 * @author Jon Woolfrey
 * @data   June 2024
 * @brief  A class for subscribing to the joint state topic of a robot arm in ROS2.
 */
 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <RobotLibrary/KinematicTree.h>

using JointState = sensor_msgs::msg::JointState;

/**
 * This class / node will subscribe to a prescribed joint state topic and update the robot
 * kinematics and dynamics as it receives messages.
 */
class JointStateSubscriber : public rclcpp::Node
{
    public:
        
        JointStateSubscriber(KinematicTree *model,
                             SerialKinematicControl *controller,
                             const std::string &topicName = "unnamed")
        : Node("joint_state_subscriber"),
          _model(model),
          _controller(controller),
          _topicName(topicName)
        {
            using namespace std::placeholders;                                                      // Used when binding callback function to subscriber
            
            this->_subscription =
            this->create_subscription<JointState>(this->_topicName, 10,
                                                  std::bind(&JointStateSubscriber::callback,
                                                  this, _1));
        
            std::string message = "Subscribing to '" + this->_topicName + "' joint state topic.";
            
            RCLCPP_INFO(this->get_logger(), message.c_str());
        }
        
    private:

        KinematicTree *_model;                                                                      // Pointer to robot model
        
        SerialKinematicControl *_controller;
        
        std::string _topicName = "unnamed";                                                         // Topic name
        
        rclcpp::Subscription<JointState>::SharedPtr _subscription;                                  // This is the fundamental object
        
        void callback(const JointState &state)
        {
            if(this->_model->update_state(Eigen::Map<const Eigen::VectorXd>(state.position.data(), state.position.size()),
                                          Eigen::Map<const Eigen::VectorXd>(state.velocity.data(), state.velocity.size())))
            {
                this->_controller->update();
            }                 
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to update robot state.");
                
                // NOTE: Need to do something here to stop any controllers that are running!
            }
        }
};

