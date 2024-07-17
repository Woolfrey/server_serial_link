/**
 * @file   JointStateSubscriber.h
 * @author Jon Woolfrey
 * @data   June 2024
 * @brief  A class for subscribing to the joint state topic of a robot arm in ROS2.
 */
 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/wait_for_message.hpp"

using JointState = sensor_msgs::msg::JointState;

/**
 * This class will subscribe to a given joint state topic
 * and update the kinematics and dynamics of the model accordingly.
 */
class JointStateSubscriber : public rclcpp::Node
{
    public:
        
        JointStateSubscriber() : Node("joint_state_subscriber")
        {
            using namespace std::placeholders;                                                      // Used when binding callback function to subscriber
            
            this->_subscription =
            this->create_subscription<JointState>("topic", 10,
                                                  std::bind(&JointStateSubscriber::callback,
                                                  this, _1));
        }
        
    private:
        
        std::string _topicName = "topic";
        
        rclcpp::Subscription<JointState>::SharedPtr _subscription;
        
        void callback(const JointState &state)
        {
            (void)state;                                                                            // Temporary
            
            // this->_model->update_state(Eigen::VectorXd(state->position.data()),
            //                            Eigen::VectorXd(state->velocity.data()));                             
        }
};

