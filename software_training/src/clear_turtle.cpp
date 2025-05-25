#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace software_training{

class ClearTurtle : public rclcpp::Node {
    public:
        using ClearService = std_srvs::srv::Empty;

        explicit ClearTurtle(const rclcpp::NodeOptions & options)
            : Node("clear_turtle_comp", options) {

            this->client = this->create_client<ClearService>("/clear");

            this->timer = this->create_wall_timer(
            std::chrono::milliseconds(300),
            std::bind(&ClearTurtle::send_clear_request, this));

            RCLCPP_INFO(this->get_logger(), "ClearTurtle node started, will attempt to clear turtlesim.");
        }

    private:
        rclcpp::Client<ClearService>::SharedPtr client;
        rclcpp::TimerBase::SharedPtr timer;

        void send_clear_request() {

            while (!client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            this->timer->cancel();
            auto request = std::make_shared<ClearService::Request>();
            auto result = client->async_send_request(request,
                [this](rclcpp::Client<ClearService>::SharedFuture future) {
                    try
                    {
                        future.get();
                        RCLCPP_INFO(this->get_logger(), "Turtles cleared!");
                    }
                    catch(const std::exception& e)
                    {
                        RCLCPP_INFO(this->get_logger(), "Failed to clear turtles");
                    }
                    rclcpp::shutdown();
                }
            );
        }
};

} // namespace software_training

RCLCPP_COMPONENTS_REGISTER_NODE(software_training::ClearTurtle)
