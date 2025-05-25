#include <chrono>
#include <memory>
#include <functional>
#include <cmath>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "software_training/action/moving_turtle.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#define kp 2

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace software_training {

double normalize_angle(float angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

class MovingTurtleServer : public rclcpp::Node {
    public:
        explicit MovingTurtleServer(const rclcpp::NodeOptions &options)
            : Node("moving_turtle_server", options) {
            
            this->twist_publisher = this->create_publisher<Twist>("/moving_turtle/cmd_vel", 10);
            this->pose_subscriber = this->create_subscription<Pose>("/moving_turtle/pose", 10, 
                std::bind(&MovingTurtleServer::moving_turtle_pose_callback, this, _1));
            this->action_server = rclcpp_action::create_server<MovingTurtle>(
                this,
                "moving_turtle",
                std::bind(&MovingTurtleServer::handle_goal, this, _1, _2),
                std::bind(&MovingTurtleServer::handle_cancel, this, _1),
                std::bind(&MovingTurtleServer::handle_accepted, this, _1));

            RCLCPP_INFO(this->get_logger(), "MovingTurtleServer started successfully and is waiting for request!");
        }

    private:
        using Twist = geometry_msgs::msg::Twist;
        using Pose = turtlesim::msg::Pose;
        using MovingTurtle = software_training::action::MovingTurtle;
        using GoalHandleMovingTurtle = rclcpp_action::ServerGoalHandle<MovingTurtle>;

        rclcpp::Publisher<Twist>::SharedPtr twist_publisher;
        rclcpp::Subscription<Pose>::SharedPtr pose_subscriber;
        rclcpp_action::Server<MovingTurtle>::SharedPtr action_server;

        Pose::SharedPtr moving_turtle_pose;

        void moving_turtle_pose_callback(const Pose::SharedPtr pose) {
            moving_turtle_pose = pose;
        }

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const MovingTurtle::Goal> goal) {
            
                RCLCPP_INFO(this->get_logger(), "Received goal request with x: %.2f and y: %.2f", goal->target_x, goal->target_y);
                (void)uuid;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMovingTurtle> goal_handle) {
            (void)goal_handle;
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleMovingTurtle> goal_handle) {
            std::thread{std::bind(&MovingTurtleServer::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleMovingTurtle> goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Executing goal");
            rclcpp::Time start = this->now();
            rclcpp::Rate rate{5};

            auto result = std::make_shared<MovingTurtle::Result>();

            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<MovingTurtle::Feedback>();

            bool rotate_goal = false;

            while(rclcpp::ok()) {
                if (goal_handle->is_canceling()) {
                    rclcpp::Time end = this->now();
                    rclcpp::Duration timer_length = end - start;
                    result->duration = timer_length.nanoseconds();
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }

                float delta_x = goal->target_x - moving_turtle_pose->x;
                float delta_y = goal->target_y - moving_turtle_pose->y;

                float distance = std::hypot(delta_x, delta_y);

                // calculate desired angle turtle should be at to reach goal in a straight line
                float ref_angle = normalize_angle(atan2(delta_y, delta_x));
                float theta_error = ref_angle - moving_turtle_pose->theta;

                auto twist_msg = Twist();

                if(distance < 0.05) {
                    twist_msg.linear.x = 0.0;
                    twist_msg.angular.z = 0.0;
                    twist_publisher->publish(twist_msg);
                    rclcpp::Time end = this->now();
                    rclcpp::Duration timer_length = end - start;
                    result->duration = timer_length.nanoseconds();
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                    return;
                }

                if(fabs(theta_error) > 0.01) {
                    twist_msg.angular.z = kp * theta_error;
                }
                else {
                    twist_msg.angular.z = 0;
                    rotate_goal = true;
                }

                if(rotate_goal) {
                    twist_msg.linear.x = kp * distance;
                }

                twist_publisher->publish(twist_msg);

                feedback->x_distance = delta_x;
                feedback->y_distance = delta_y;

                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Published feedback x: %.2f, y: %.2f", feedback->x_distance, feedback->y_distance);

                rate.sleep();
            }
        }
};

} // namespace software_training

RCLCPP_COMPONENTS_REGISTER_NODE(software_training::MovingTurtleServer)
