/**
 * @file   ModelUpdater.h
 * @author Jon Woolfrey
 * @data   June 2024
 * @brief  This node subscribes to the joint state topic and updates the kinematics, dynamics.
 */
 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <RobotLibrary/KinematicTree.h>

using JointState = sensor_msgs::msg::JointState;

/**
 * This class / node will subscribe to a prescribed joint state topic and update the robot
 * kinematics and dynamics as it receives messages.
 */
class ModelUpdater : public rclcpp::Node
{
    public:
        
        /**
         * Constructor.
         * @param model A pointer to the kinematic/dynamic model of the robot.
         * @param topicName The name of the joint state topic to subscribe to.
         */
        ModelUpdater(KinematicTree *model, const std::string &topicName = "joint_states");
        
    private:

        KinematicTree *_model;                                                                      ///< Pointer to robot model.
        
        std::string _topicName = "joint_states";                                                    ///< Name of joint state topic to subscribe to
        
        rclcpp::Subscription<JointState>::SharedPtr _subscription;                                  ///< This is the fundamental object
        
        /**
         * Callback function that updates the kinematics & dynamics of the model.
         * @param state The joint state message containing position and velocity.
         */
        void
        update(const JointState &state);
};

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //     
////////////////////////////////////////////////////////////////////////////////////////////////////
ModelUpdater::ModelUpdater(KinematicTree *model, const std::string &topicName)
                           : Node(model->name() + "_model_updater"),
                             _model(model),
                             _topicName(topicName)
{
    using namespace std::placeholders;                                                              // Used when binding callback function to subscriber
    
    _subscription = this->create_subscription<JointState>(this->_topicName, 1, std::bind(&ModelUpdater::update, this, _1));

    std::string message = "Subscribing to '" + this->_topicName + "' joint state topic.";
    
    RCLCPP_INFO(this->get_logger(), message.c_str());
    
    model->update_state(Eigen::VectorXd::Zero(model->number_of_joints()),
                        Eigen::VectorXd::Zero(model->number_of_joints()));
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Callback function                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
ModelUpdater::update(const JointState &state)
{
    if(state.position.size() != _model->number_of_joints()
    or state.velocity.size() != _model->number_of_joints())
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Dimensions for joint state does not match number of joints in model.");
        
        return;
    }
    else if(not this->_model->update_state(Eigen::Map<const Eigen::VectorXd>(state.position.data(), state.position.size()),
                                           Eigen::Map<const Eigen::VectorXd>(state.velocity.data(), state.velocity.size())))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Failed to update robot state.");
        
        return;
    }
}