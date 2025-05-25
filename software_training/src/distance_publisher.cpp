#include <chrono>
#include <memory>
#include <functional>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "software_training/msg/distance.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace software_training {

class DistancePublisher : public rclcpp::Node {
    public:
    
        explicit DistancePublisher(const rclcpp::NodeOptions &options)
            : Node("distance_publisher", options) {

            this->moving_turtle_subscriber = this->create_subscription<Pose>("/moving_turtle/pose", 10, 
                std::bind(&DistancePublisher::moving_turtle_callback, this, _1));
            this->stationary_turtle_subscriber = this->create_subscription<Pose>("/stationary_turtle/pose", 10, 
                std::bind(&DistancePublisher::stationary_turtle_callback, this, _1));

            this->publisher = this->create_publisher<Distance>("/moving_to_stationary_distance", 10);
            this->timer = this->create_wall_timer(1s, std::bind(&DistancePublisher::publish_distance, this));
        }

    private:
        using Pose = turtlesim::msg::Pose;
        using Distance = software_training::msg::Distance;

        rclcpp::TimerBase::SharedPtr timer;

        rclcpp::Subscription<Pose>::SharedPtr moving_turtle_subscriber;
        rclcpp::Subscription<Pose>::SharedPtr stationary_turtle_subscriber;
        rclcpp::Publisher<Distance>::SharedPtr publisher;

        Pose::SharedPtr moving_turtle_pose;
        Pose::SharedPtr stationary_turtle_pose;

        void moving_turtle_callback(const Pose::SharedPtr pose) {
            moving_turtle_pose = pose;
        }
        void stationary_turtle_callback(const Pose::SharedPtr pose) {
            stationary_turtle_pose = pose;
        }

        void publish_distance() {
            auto message = Distance();
            message.x_distance = fabs(moving_turtle_pose->x - stationary_turtle_pose->x);
            message.y_distance = fabs(moving_turtle_pose->y - stationary_turtle_pose->y);
            message.distance = sqrt(message.x_distance*message.x_distance + message.y_distance*message.y_distance);

            RCLCPP_INFO(this->get_logger(), "Publishing: x: %.2f, y: %.2f, distance: %.2f", message.x_distance, message.y_distance, message.distance);
            publisher->publish(message);
        }
};
} // namespace software_training

RCLCPP_COMPONENTS_REGISTER_NODE(software_training::DistancePublisher)
