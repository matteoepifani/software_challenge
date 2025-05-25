#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "software_training/action/moving_turtle.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace software_training {
class MovingTurtleClient : public rclcpp::Node {
    public:
        explicit MovingTurtleClient(const rclcpp::NodeOptions &options)
            : Node("moving_turtle_client", options) {

            this->declare_parameter<float>("x", 3.0);
            this->declare_parameter<float>("y", 4.0);

            target[0] = (float)this->get_parameter("x").as_double();
            target[1] = (float)this->get_parameter("y").as_double();

            this->client = rclcpp_action::create_client<MovingTurtle>(this, "moving_turtle");

            this->timer = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&MovingTurtleClient::send_goal, this));

        }

    private:
        using MovingTurtle = software_training::action::MovingTurtle;
        using GoalHandleMovingTurtle = rclcpp_action::ClientGoalHandle<MovingTurtle>;

        rclcpp_action::Client<MovingTurtle>::SharedPtr client;
        rclcpp::TimerBase::SharedPtr timer;

        float target[2];

        void send_goal() {
            this->timer->cancel();
            if (!this->client->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                rclcpp::shutdown();
            }

            auto goal_msg = MovingTurtle::Goal();
            goal_msg.target_x = target[0];
            goal_msg.target_y = target[1];

            RCLCPP_INFO(this->get_logger(), "Sending goal x: %.2f, y: %.2f", goal_msg.target_x, goal_msg.target_y);

            auto send_goal_options = rclcpp_action::Client<MovingTurtle>::SendGoalOptions();
            send_goal_options.goal_response_callback =
            std::bind(&MovingTurtleClient::goal_response_callback, this, _1);
            send_goal_options.feedback_callback =
            std::bind(&MovingTurtleClient::feedback_callback, this, _1, _2);
            send_goal_options.result_callback =
            std::bind(&MovingTurtleClient::result_callback, this, _1);
            this->client->async_send_goal(goal_msg, send_goal_options);
        }

        void goal_response_callback(std::shared_future<GoalHandleMovingTurtle::SharedPtr> future) {
            auto goal_handle = future.get();
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        }

        void feedback_callback(GoalHandleMovingTurtle::SharedPtr, const std::shared_ptr<const MovingTurtle::Feedback> feedback) {
            RCLCPP_INFO(this->get_logger(), "Feedback distance x: %.2f, y: %.2f", feedback->x_distance, feedback->y_distance);
        }

        void result_callback(const GoalHandleMovingTurtle::WrappedResult & result) {
            switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Result received in %lu nanoseconds", result.result->duration);
            rclcpp::shutdown();
        }
};

} // namespace software_training

RCLCPP_COMPONENTS_REGISTER_NODE(software_training::MovingTurtleClient)
