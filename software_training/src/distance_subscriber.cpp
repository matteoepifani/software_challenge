#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "software_training/msg/distance.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::placeholders;

namespace software_training {

class DistanceSubscriber : public rclcpp::Node {
    public:
    
        explicit DistanceSubscriber(const rclcpp::NodeOptions &options)
            : Node("distance_subscriber", options) {

            this->subscribtion = this->create_subscription<Distance>("/moving_to_stationary_distance", 10, 
                std::bind(&DistanceSubscriber::subscription_callback, this, _1));
        }

    private:
        using Distance = software_training::msg::Distance;
        rclcpp::Subscription<Distance>::SharedPtr subscribtion;

        void subscription_callback(const Distance::SharedPtr message) {
            RCLCPP_INFO(this->get_logger(), "Listened: x: %.2f, y: %.2f, distance: %.2f", message->x_distance, message->y_distance, message->distance);
        }
};
} // namespace software_training

RCLCPP_COMPONENTS_REGISTER_NODE(software_training::DistanceSubscriber)
