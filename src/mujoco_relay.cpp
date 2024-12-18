/**
 * @file   mujoco_relay.cpp
 * @author Jon Woolfrey
 * @data   November 2024
 * @brief  A ROS2 node that converts joint commands to be used by the mujoco_ros2_interface package.
 */

#include "rclcpp/rclcpp.hpp"
#include "serial_link_action_server/msg/joint_command.hpp"                                          // Custom message
#include "std_msgs/msg/float64_multi_array.hpp"                                                     // For joint_commands topic

class MujocoRelayNode : public rclcpp::Node
{
    public:
        
        // For brevity
        using JointCommandMsg = serial_link_action_server::msg::JointCommand;
        using Float64MultiArray = std_msgs::msg::Float64MultiArray;
        
        /**
         * Constructor.
         * @nodeName As it says.
         * @subscriptionName Name of the joint command topic.
         * @publicationName Name for new topic to publish to.
         */
        MujocoRelayNode(const std::string &nodeName = "mujoco_relay",
                        const std::string &subscriptionName = "joint_command_relay",
                        const std::string &publicationName = "joint_commands")
                        : Node(nodeName)
        {
            _jointCommandSubscriber = this->create_subscription<JointCommandMsg>
            (
                subscriptionName,                                                                   // Name of topic to subscribe to
                1,                                                                                  // No. of msgs to retain
                std::bind(&MujocoRelayNode::relay, this, std::placeholders::_1)                     // Callback function for when message is received
            );

            _jointCommandPublisher = this->create_publisher<Float64MultiArray>(publicationName, 1); 
        }

    private:

        rclcpp::Subscription<JointCommandMsg>::SharedPtr _jointCommandSubscriber;                   ///< Subscriber for custom JointCommandMsg
        
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _jointCommandPublisher;      ///< Publisher for array of joint commands
        
        /*
         * This method extracts the joint commands from the custom message and publishes it as an array.
         * @param msg The JointCommandMsg message type.
         */
        void relay(const JointCommandMsg::SharedPtr msg)
        {
            Float64MultiArray relayMsg;                                                             // Create Float64MultiArray on the stack

            relayMsg.data.reserve(msg->command.size());                                             // Reserve memory to avoid multiple allocations if size of command array is known

            relayMsg.data = std::move(msg->command);                                                // Use std::move to move data from the message's command vector if you know you don't need msg->command later

            _jointCommandPublisher->publish(relayMsg);                                              // Publish the message (no shared pointer required)
        }
};

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           MAIN                                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                                                                       // Start up ROS2
    rclcpp::spin(std::make_shared<MujocoRelayNode>());                                              // Run the node
    rclcpp::shutdown();                                                                             // As it says
    return 0;
}

