#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace software_training{

class SpawnTurtle : public rclcpp::Node {
    public:
        explicit SpawnTurtle(const rclcpp::NodeOptions & options)
            : Node("spawn_turtle_comp", options) {

            this->client = this->create_client<SpawnService>("/spawn");

            this->timer = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SpawnTurtle::send_spawn_request, this));

            RCLCPP_INFO(this->get_logger(), "SpawnTurtle node started, will attempt to spawn turtles.");
        }

    private:
        using SpawnService = turtlesim::srv::Spawn;
        rclcpp::Client<SpawnService>::SharedPtr client;
        rclcpp::TimerBase::SharedPtr timer;

        struct turtle_info {
            std::string name;
            float x;
            float y;
            float theta;
        };

        turtle_info turtles_to_spawn[2] = {
            {"stationary_turtle", 5, 5, 0},
            {"moving_turtle", 7, 8, 0}
        };

        void send_spawn_request() {

            while (!client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            this->timer->cancel();

            for (turtle_info turtle : turtles_to_spawn) {
                auto request = std::make_shared<SpawnService::Request>();

                request->name = turtle.name;
                request->x = turtle.x;
                request->y = turtle.y;
                request->theta = turtle.theta;

                std::string requested_turtle = turtle.name;

                auto result = client->async_send_request(request,
                    [this, requested_turtle](rclcpp::Client<SpawnService>::SharedFuture future) {
                        try
                        {
                            future.get();
                            RCLCPP_INFO(this->get_logger(), "Spawned %s", requested_turtle.c_str());
                        }
                        catch(const std::exception& e)
                        {
                            RCLCPP_INFO(this->get_logger(), "Failed to spawn %s", requested_turtle.c_str());
                        }

                        if (requested_turtle == "moving_turtle") {
                            rclcpp::shutdown();
                        }
                    }
                );
            }    

        }
};

} // namespace software_training

RCLCPP_COMPONENTS_REGISTER_NODE(software_training::SpawnTurtle)
