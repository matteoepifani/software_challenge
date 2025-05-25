#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace software_training {

class Turtle1Circle : public rclcpp::Node {

    public:

        explicit Turtle1Circle(const rclcpp::NodeOptions & options)
            : Node("turtle1_circle", options) {

            this->publisher = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

            this->timer = this->create_wall_timer(1ms, 
                [this](void) {
                    auto message = std::make_unique<geometry_msgs::msg::Twist>();
                    message->linear.x = 2;
                    message->linear.y = 0;
                    message->linear.z = 0;

                    message->angular.x = 0;
                    message->angular.y = 0;
                    message->angular.z = 2;

                    RCLCPP_INFO(this->get_logger(), "Publishing message.");
                    
                    publisher->publish(std::move(message));
                }
            );
        }

    private:
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

};

} // namespace software_training

RCLCPP_COMPONENTS_REGISTER_NODE(software_training::Turtle1Circle)