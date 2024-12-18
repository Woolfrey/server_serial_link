/**
 * @file   ActionClientBase.h
 * @author Jon Woolfrey
 * @data   October 2024
 * @brief  This is a base class for action clients in ROS2.
 */

#ifndef ACTION_CLIENT_BASE_H
#define ACTION_CLIENT_BASE_H

#include <client/ActionClientInterface.h>
#include <memory>
#include <string>
#include <action_msgs/srv/cancel_goal.hpp>

template <class Action>
class ActionClientBase : public ActionClientInterface
{
    public:
        
        using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;                                 ///< For easier referencing

        /**
         * Constructor.
         * @param clientNode A pointer to the client node.
         * @param actionName Must match what is advertised by the server.
         */
        ActionClientBase(std::shared_ptr<rclcpp::Node> clientNode,
                         const std::string &actionName);

        /**
         * Send a request to execute an action.
         * @param goal The goal field of the action to be sent to the server.
         * @return A std::shared_future to the goal handle.
         */
        bool
        send_request(const typename Action::Goal::SharedPtr &goal);

        /**
         * It does what you'd expect.
         * @return False if no action is running.
         */
        bool
        cancel_action() override;
        
        /**
         * This overrides the method defined in the base class.
         * https://docs.ros2.org/foxy/api/action_msgs/msg/GoalStatus.html
         * 0 = Unknown
         * 1 = Accepted
         * 2 = Executing
         * 3 = Canceling
         * 4 = Succeeded
         * 5 = Canceled
         * 6 = Aborted
         * @return An int8_t for the status.
         */
        int8_t
        status() const override
        {
            if(_goalHandle) return _goalHandle->get_status();
            else            return 0;
        }
        
        /**
         * Checks to see if an action is currently active.
         * @return True if it is it is accepted (about to start), currently executing, or in the process of canceling.
         */
        bool
        is_running() const override
        {
            if(status() == 1    
            or status() == 2
            or status() == 3)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
 
    protected:
        
        std::shared_ptr<rclcpp::Node> _node;                                                        ///< Pointer to client node.
        
        typename rclcpp_action::Client<Action>::SendGoalOptions _options;                           ///< These are used to set callback functions        
       
        typename rclcpp_action::Client<Action>::SharedPtr _actionClient;                            ///< This is the foundation of the class

        typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr _goalHandle;                    ///< Current goal handle
          
        /**
         * This method is used to process the response from the action server after sending a goal.
         * @param GoalHandle A pointer to the goal handle associated with the action.
         */
        void
        response_callback(const typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr goalHandle);
        
        /**
         * This method executes when an action is finished
         * @param result The result portion of the associated goal field.
         */
        void
        result_callback(const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result);
            
        /**
         * This method executes after an action server has completed the cancellation process.
         */
        void
        cancel_callback(const typename rclcpp_action::Client<Action>::CancelResponse::SharedPtr response);
};

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           Constructor                                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
ActionClientBase<Action>::ActionClientBase(std::shared_ptr<rclcpp::Node> clientNode,
                                                   const std::string &actionName)
                                                   : ActionClientInterface(),
                                                     _node(clientNode),
                                                     _actionClient(rclcpp_action::create_client<Action>(_node, actionName))
{
    // Attach the response callback after an action request is sent
    _options.goal_response_callback = std::bind
    (
        &ActionClientBase::response_callback,                                                       // Name of the method
        this,                                                                                       // Attach this node
        std::placeholders::_1                                                                       // I don't know what this does
    );

    // Attach the result callback for when an action is finished
    _options.result_callback = std::bind
    (
        &ActionClientBase::result_callback,                                                         // Name of the method
        this,                                                                                       // Attach this node
        std::placeholders::_1                                                                       // I don't know what this does
    );
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Send an action request to the server                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
bool
ActionClientBase<Action>::send_request(const typename Action::Goal::SharedPtr &goal)
{
    auto goalHandleFuture = _actionClient->async_send_goal(*goal, _options);                        // Send request, and link callback methods
    
    if (goalHandleFuture.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready)
    {
        return goalHandleFuture.get() ? true : false;                                               // We want a fast return
    }
    else
    {
        RCLCPP_INFO(_node->get_logger(), "Failed to receive response from server after 500 ms.");
        
        return false;
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Processes the response to an action request                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
void
ActionClientBase<Action>::response_callback(const typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr goalHandle)
{
    if(goalHandle)                                                                                  // Not a null pointer
    {
        _goalHandle = goalHandle;                                                                   // Save it internally
    }
    else
    {
        RCLCPP_INFO(_node->get_logger(), "Action request rejected by the server.");
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                  Executes when an action is finished.                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
void
ActionClientBase<Action>::result_callback(const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result)
{
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            RCLCPP_INFO(_node->get_logger(), "Action completed.");
            break;
        }
        case rclcpp_action::ResultCode::CANCELED:
        {
            RCLCPP_INFO(_node->get_logger(), "Action canceled.");
            break;
        }
        case rclcpp_action::ResultCode::ABORTED:
        {
            RCLCPP_INFO(_node->get_logger(), "Action aborted.");
            break;
        }
        default:
        {
            RCLCPP_WARN(_node->get_logger(), "Unknown result code.");
            break;
        }
    }
} 

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Cancel the action that is in progress                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
bool
ActionClientBase<Action>::cancel_action()
{
    auto cancelFuture = _actionClient->async_cancel_goal
    (
        _goalHandle,
        [this](const typename rclcpp_action::Client<Action>::CancelResponse::SharedPtr response)
        {
            this->cancel_callback(response);
        }
    );

    if (cancelFuture.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready)
    {
        return true;
    }
    else
    {
        RCLCPP_WARN(_node->get_logger(), "Request for action cancellation timed out.");
        return false;
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Executes after cancelling                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Action>
void
ActionClientBase<Action>::cancel_callback(const typename rclcpp_action::Client<Action>::CancelResponse::SharedPtr cancelResponse)
{
    (void)cancelResponse;
    
    // NEED TO FILL THIS IN
}

#endif
