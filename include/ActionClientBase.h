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
        
        using ActionManager = rclcpp_action::ClientGoalHandle<Action>;

        /**
         * Constructor.
         * @param clientNode A pointer to the client node.
         * @param actionName Must match what is advertised by the server.
         */
        ActionClientBase(std::shared_ptr<rclcpp::Node> clientNode,
                         const std::string &actionName)
                         : _node(clientNode),
                           _actionClient(rclcpp_action::create_client<Action>(_node, actionName))
        {
            // Worker bees can leave.
            // Even drones can fly away.
            // The Queen is their slave.
        }

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

            // Create SendGoalOptions
            typename rclcpp_action::Client<Action>::SendGoalOptions options;

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

            // Send the goal
            auto goalHandleFuture = _actionClient->async_send_goal(*goal, options);
        }
    protected:
        
        std::shared_ptr<rclcpp::Node> _node;                                                        // Pointer to client node.
       
        typename rclcpp_action::Client<Action>::SharedPtr _actionClient;                            // This is the foundation of the class

        /**
         * Processes the response to an action request.
         * This is a pure virtual function and must be defined in any derived class.
         * @param future A future containing the result of the goal send operation.
         */
        virtual void process_response(const typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr &goalHandle) = 0;

        /**
         * Processes the result of an action.
         * This is a pure virtual function and must be elaborated on in any derived class.
         * @param result The wrapped result of the action.
         */
        virtual void process_result(const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result) = 0;
};

#endif // ACTION_CLIENT_BASE_H

