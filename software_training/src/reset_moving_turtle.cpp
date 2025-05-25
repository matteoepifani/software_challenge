#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "software_training/srv/reset.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace software_training {

class ResetMovingTurtle : public rclcpp::Node {
    public:
        using Reset = software_training::srv::Reset;
        using Teleport = turtlesim::srv::TeleportAbsolute;

        explicit ResetMovingTurtle(const rclcpp::NodeOptions &options)
            : Node("clear_turtle_comp", options) {

            this->client = this->create_client<Teleport>("/moving_turtle/teleport_absolute");
            this->service = this->create_service<Reset>("/reset_moving_turtle",
                std::bind(&ResetMovingTurtle::reset_callback, this, _1, _2));

            RCLCPP_INFO(this->get_logger(), "Waiting for client request.");
        }

    private:
        rclcpp::Client<Teleport>::SharedPtr client;
        rclcpp::Service<Reset>::SharedPtr service;

        void reset_callback(const std::shared_ptr<Reset::Request> request,
            std::shared_ptr<Reset::Response> response) {

            (void)request;

            while (!client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    response->success = false;
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto tp_request = std::make_shared<Teleport::Request>();

            tp_request->x = 25;
            tp_request->y = 10;
            tp_request->theta = 0;

            auto result = client->async_send_request(tp_request,
                [this, response](rclcpp::Client<Teleport>::SharedFuture future) {
                    try
                    {
                        future.get();
                        RCLCPP_INFO(this->get_logger(), "Reset moving_turtle!");
                    }
                    catch(const std::exception& e)
                    {
                        RCLCPP_INFO(this->get_logger(), "Failed to reset moving_turtle");
                    }
                }
            );
            response->success = true;
        }
};

} // namespace software_training

RCLCPP_COMPONENTS_REGISTER_NODE(software_training::ResetMovingTurtle)
