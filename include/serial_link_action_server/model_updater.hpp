/**
 * @file    model_updater.hpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   A class responsible for updating the kinematics & dynamics of a KinematicTree model.
 * 
 * @details This class uses a ROS2 node to subscribe to a specified joint state topic.
 *          When a new joint state is received, it updates the forward kinematics and inverse dynamics
 *          of the associated KinematicTree object. It is designed to run independently to a controler.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information on the KinematicTree class.
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <RobotLibrary/Model/KinematicTree.h>

using JointState = sensor_msgs::msg::JointState;                                                    // For brevity

namespace serial_link_action_server {

/**
 * @brief Subscribes to a ROS2 sensor_msgs/msg/JointState topic, and updates kinematics & dynamics.
 */
class ModelUpdater : public rclcpp::Node
{
    public:
        
        /**
         * @brief Constructor.
         * @param model A pointer to the kinematic/dynamic model of the robot.
         * @param topicName The name of the joint state topic to subscribe to.
         */
        ModelUpdater(std::shared_ptr<RobotLibrary::Model::KinematicTree> model,
                     const std::string &topicName = "joint_states");
        
    private:

        std::shared_ptr<RobotLibrary::Model::KinematicTree> _model;                                 ///< Pointer to robot model.
        
        std::string _topicName = "joint_states";                                                    ///< Name of joint state topic to subscribe to
        
        rclcpp::Subscription<JointState>::SharedPtr _subscription;                                  ///< This is the fundamental object
        
        /**
         * @brief Callback function that updates the kinematics & dynamics of the model.
         * @param state The joint state message containing position and velocity.
         */
        void
        update(const JointState &state);
};

}
