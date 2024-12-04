#ifndef CARTESIAN_TRAJECTORY_CLIENT_H
#define CARTESIAN_TRAJECTORY_CLIENT_H

#include "ActionClientBase.h"
#include "serial_link_action_server/action/track_cartesian_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

class CartesianTrajectoryClient : public ActionClientBase<serial_link_action_server::action::TrackCartesianTrajectory>
{
    public:
    
        using Action = serial_link_action_server::action::TrackCartesianTrajectory;
    
        CartesianTrajectoryClient(std::shared_ptr<rclcpp::Node> clientNode,
                                  const std::string &actionName)
                                : ActionClientBase(clientNode, actionName) {}
};

#endif

