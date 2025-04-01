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
                           const std::string &topicName,
                           const std::string &endpointName)
: Node(model->name() + "_model_updater"),
  _model(model),
  _endpointName(endpointName)
{  
    _subscription = this->create_subscription<JointState>(topicName, 1, std::bind(&ModelUpdater::update, this, std::placeholders::_1));

    _transformBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
          
    // Set initial state
    _model->update_state(Eigen::VectorXd::Zero(model->number_of_joints()),
                         Eigen::VectorXd::Zero(model->number_of_joints()));      
    
    // Look up the endpoint frame
    try
    {
        _endpointFrame = _model->find_frame(endpointName);
    }
    catch (const std::exception &exception)
    {
        RCLCPP_WARN(this->get_logger(), exception.what());
    }
    
    // Save these since they're fixed
    _transform.header.frame_id = _model->base.name();
    _transform.child_frame_id  = _endpointName;
    
    if (not (_endpointFrame->link == nullptr))
    {
        RCLCPP_INFO(this->get_logger(),
                    "Initiated the model updater node for the `%s`. "
                    "Subscribing to the `%s` joint state topic. "
                    "Publishing transform for the `%s` frame.",
                    _model->name().c_str(), topicName.c_str(), _endpointName.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),
                    "Initiated the model updater node for the `%s`. "
                    "Subscribing to the `%s` joint state topic.",
                    _model->name().c_str(), topicName.c_str());
    }
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
    
    // Publish endpoint transform:
    if (not(_endpointFrame == nullptr))
    {
        _transform.header.stamp = this->now();                                                      // Add current time stamp
        
        // Transfer the pose
        _transform.transform.translation.x = _endpointFrame->link->pose().translation()[0];
        _transform.transform.translation.y = _endpointFrame->link->pose().translation()[1];
        _transform.transform.translation.z = _endpointFrame->link->pose().translation()[2];
        
        _transform.transform.rotation.w = _endpointFrame->link->pose().quaternion().w();
        _transform.transform.rotation.x = _endpointFrame->link->pose().quaternion().x();
        _transform.transform.rotation.y = _endpointFrame->link->pose().quaternion().y();
        _transform.transform.rotation.z = _endpointFrame->link->pose().quaternion().z();
            
        _transformBroadcaster->sendTransform(_transform);
    }
}

} // namespace
