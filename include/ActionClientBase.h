#ifndef ACTION_CLIENT_BASE_H
#define ACTION_CLIENT_BASE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <string>

template <class Action>
class ActionClientBase
{
    public:
        
        using ActionManager = rclcpp_action::ClientGoalHandle<Action>;                              ///< For easier referencing

        /**
         * Constructor.
         * @param clientNode A pointer to the client node.
         * @param actionName Must match what is advertised by the server.
         */
        ActionClientBase(std::shared_ptr<rclcpp::Node> clientNode,
                         const std::string &actionName)
                         : _node(clientNode),
                           _actionClient(rclcpp_action::create_client<Action>(_node, actionName)) {}

        /**
         * Send a request to execute an action.
         * @param goal The goal field of the action to be sent to the server.
         */
        void send_request(const typename Action::Goal::SharedPtr &goal)
        {
            using namespace std::placeholders;

            if (!_actionClient->wait_for_action_server(std::chrono::seconds(1)))
            {
                RCLCPP_ERROR(_node->get_logger(), "Action server not available.");
                return;
            }

            typename rclcpp_action::Client<Action>::SendGoalOptions options;                        // These are used to set callback functions

            // Set up the callback for processing the goal response
            options.goal_response_callback = std::bind(
                &ActionClientBase::process_response, 
                this, 
                std::placeholders::_1
            );

            // Set up the callback for processing the result
            options.result_callback = std::bind(
                &ActionClientBase::process_result, 
                this, 
                std::placeholders::_1
            );

            auto goalHandleFuture = _actionClient->async_send_goal(*goal, options);                 // Send the goal
        }

        /**
         *  Does this really need an explanation?
         * @return True or false.
         */
        bool
        action_is_running() const { return _actionManager != nullptr; }                             // Returns true if there is an active goal
 
        /**
         * It does what you'd expect.
         * @return False if no action is running.
         */
        bool cancel_action()
        {
            if (_actionManager)
            {
                auto cancel_future = _actionClient->async_cancel_goal(_actionManager);
                
                if (cancel_future.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready)
                {
                    _actionManager.reset();
                }
                else
                {
                    RCLCPP_WARN(_node->get_logger(), "Cancellation request timed out.");
                }

                return true;
            }
            else
            {
                RCLCPP_WARN(_node->get_logger(), "No action is currently running.");
                return false;
            }
        }
 
    protected:
        
        std::shared_ptr<rclcpp::Node> _node;                                                        // Pointer to client node.
       
        typename rclcpp_action::Client<Action>::SharedPtr _actionClient;                            // This is the foundation of the class

        typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr _actionManager;                 // Current goal handle
        
        /**
         * Processes the response to an action request.
         * @param future A future containing the result of the goal send operation.
         */
        void process_response(const typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr &actionManager)
        {
            if (actionManager)
            {
                _actionManager = actionManager;                                                     // Store the goal handle
            }
            else
            {
                RCLCPP_INFO(_node->get_logger(), "Action request rejected.");
            }
        }

        /**
         * Processes the result of an action.
         * @param result The wrapped result of the action.
         */
        void process_result(const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result)
        {
            switch (result.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                {
                    RCLCPP_INFO(_node->get_logger(), "Action succeeded.");
                    break;
                }
                case rclcpp_action::ResultCode::CANCELED:
                {
                    RCLCPP_INFO(_node->get_logger(), "Action was canceled.");
                    break;
                }
                case rclcpp_action::ResultCode::ABORTED:
                {
                    RCLCPP_INFO(_node->get_logger(), "Action failed.");
                    break;
                }
                default:
                {
                    RCLCPP_WARN(_node->get_logger(), "Unknown action result code.");
                    break;
                }
            }

            RCLCPP_INFO(_node->get_logger(), "Resetting goal handle.");
            
            _actionManager.reset();                                                                 // Ensure the goal handle is reset after processing the result
        }
};

#endif // ACTION_CLIENT_BASE_H

