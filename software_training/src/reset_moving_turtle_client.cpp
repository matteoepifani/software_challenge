#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "software_training/srv/reset.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace software_training{

class ResetMovingTurtleClient : public rclcpp::Node {
    public:
        using Reset = software_training::srv::Reset;

        explicit ResetMovingTurtleClient(const rclcpp::NodeOptions & options)
            : Node("reset_moving_turtle_client", options) {

            this->client = this->create_client<Reset>("/reset_moving_turtle");

            this->timer = this->create_wall_timer(
            std::chrono::milliseconds(300),
            std::bind(&ResetMovingTurtleClient::send_reset_request, this));

            RCLCPP_INFO(this->get_logger(), "ResetMovingTurtleClient node started.");
        }

    private:
        rclcpp::Client<Reset>::SharedPtr client;
        rclcpp::TimerBase::SharedPtr timer;

        void send_reset_request() {

            while (!client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            this->timer->cancel();
            auto request = std::make_shared<Reset::Request>();
            auto result = client->async_send_request(request,
                [this](rclcpp::Client<Reset>::SharedFuture future) {
                    try
                    {
                        RCLCPP_INFO(this->get_logger(), "Turtle reset: %s", future.get()->success ? "true" : "false");
                    }
                    catch(const std::exception& e)
                    {
                        RCLCPP_INFO(this->get_logger(), "Failed to reset turtles");
                    }
                    rclcpp::shutdown();
                }
            );
        }
};

} // namespace software_training

RCLCPP_COMPONENTS_REGISTER_NODE(software_training::ResetMovingTurtleClient)
