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

namespace serial_link_action_server {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //     
////////////////////////////////////////////////////////////////////////////////////////////////////
ModelUpdater::ModelUpdater(std::shared_ptr<RobotLibrary::Model::KinematicTree> model,
                           const std::string &topicName,
                           const std::string &endpointName)
: Node(model->name() + "_model_updater"),
  _model(model),
  _endpointName(endpointName)
{
    _subscription = this->create_subscription<JointState>(topicName, 1, std::bind(&ModelUpdater::update, this, std::placeholders::_1));    
    
    RCLCPP_INFO(this->get_logger(), "Waiting for publishers on topic `%s`...", topicName.c_str());
    
    rclcpp::Rate rate(2);
    
    unsigned int count = 10;
    
    while (rclcpp::ok() and this->count_publishers(topicName) == 0 and count-- > 0)
    {
        rate.sleep();
    }
    
    if (this->count_publishers(topicName) == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "`%s` topic did not appear within 5 seconds of waiting.", topicName.c_str());
        
        rclcpp::shutdown();
        
        return;
    }
  
    RCLCPP_INFO(this->get_logger(),
                "Initiated the model updater node for the `%s`. "
                "Subscribing to the `%s` joint state topic.",
                _model->name().c_str(), topicName.c_str());
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
    
    try // to update the model
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

} // namespace
