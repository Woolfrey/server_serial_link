#include "rclcpp/rclcpp.hpp"
#include "serial_link_action_server/msg/joint_command.hpp"  // Include custom message
#include "std_msgs/msg/float64_multi_array.hpp"  // For joint_commands topic

class MujocoRelayNode : public rclcpp::Node
{
    public:
        
        // For brevity
        using JointCommandMsg = serial_link_action_server::msg::JointCommand;
        using Float64MultiArray = std_msgs::msg::Float64MultiArray;
        
        MujocoRelayNode(const std::string &nodeName = "mujoco_relay",
                        const std::string &subscriptionName = "joint_command_relay",
                        const std::string &publicationName = "joint_commands")
                        : Node(nodeName)
        {
            mujoco_sub_ = this->create_subscription<JointCommandMsg>
            (
                subscriptionName,                                                                   // Name of topic to subscribe to
                1,                                                                                  // No. of msgs to retain
                std::bind(&MujocoRelayNode::relay, this, std::placeholders::_1)                     // Callback function for when message is received
            );

            _jointCommandPublisher = this->create_publisher<Float64MultiArray>(publicationName, 1); // 
        }

    private:

        rclcpp::Subscription<JointCommandMsg>::SharedPtr _jointCommandSubscriber;
        
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _jointCommandPublisher;
        
        void relay(const JointCommandMsg::SharedPtr msg)
        {
            Float64MultiArray relayMsg;                                                             // Create Float64MultiArray on the stack

            relayMsg.data.reserve(msg->command.size());                                             // Reserve memory to avoid multiple allocations if size of command array is known

            relayMsg.data = std::move(msg->command);                                                // Use std::move to move data from the message's command vector if you know you don't need msg->command later

            _jointCommandPublisher->publish(relayMsg);                                              // Publish the message (no shared pointer required)
        }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MujocoRelayNode>());
    rclcpp::shutdown();
    return 0;
}

