/**
 * @file    model_updater.cpp
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

#include <serial_link_action_server/model_updater.hpp>

namespace serial_link_action_server{

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //     
////////////////////////////////////////////////////////////////////////////////////////////////////
ModelUpdater::ModelUpdater(std::shared_ptr<RobotLibrary::Model::KinematicTree> model,
                           const std::string &topicName)
                           : Node(model->name() + "_model_updater"),
                             _model(model),
                             _topicName(topicName)
{  
    _subscription = this->create_subscription<JointState>(_topicName, 1, std::bind(&ModelUpdater::update, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Subscribing to the '%s' joint state topic.", _topicName.c_str());
     
    // Set initial state
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
    
    try
    {
        _model->update_state(Eigen::Map<const Eigen::VectorXd>(state.position.data(), state.position.size()),
                             Eigen::Map<const Eigen::VectorXd>(state.velocity.data(), state.velocity.size()));
    }
    catch(const std::exception &exception)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, exception.what());
        
        return;
    }
}

}
