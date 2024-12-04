#ifndef JOINT_TRAJECTORY_CLIENT_H
#define JOINT_TRAJECTORY_CLIENT_H

#include "ActionClientBase.h"
#include "serial_link_action_server/action/track_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

class JointTrajectoryClient : public ActionClientBase<serial_link_action_server::action::TrackJointTrajectory>
{
    public:
    
        using Action = serial_link_action_server::action::TrackJointTrajectory;
    
        JointTrajectoryClient(std::shared_ptr<rclcpp::Node> clientNode,
                              const std::string &actionName)
                            : ActionClientBase(clientNode, actionName) {}
};

#endif
