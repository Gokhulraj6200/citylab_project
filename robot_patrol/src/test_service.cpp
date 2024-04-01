#include "rclcpp/client.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "custominterface/srv/get_direction.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <functional>
#include <memory>

using GetDirection = custominterface::srv::GetDirection;
using LaserScan = sensor_msgs::msg::LaserScan;
using namespace std::chrono_literals;
using namespace std::placeholders;

class DirectionClient : public rclcpp::Node {
public:
    // Methods
    DirectionClient() : Node("test_service_node") {
        client_ = this->create_client<GetDirection>("/direction_service");
        subscription_ = this->create_subscription<LaserScan>(
            "/scan",
            1,
            std::bind(&DirectionClient::scan_callback, this, _1));

        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service.");
        }
        RCLCPP_INFO(this->get_logger(), "Client started.");
    }
private:
    // Atributes
    rclcpp::Client<GetDirection>::SharedPtr client_;
    rclcpp::Subscription<LaserScan>::SharedPtr subscription_;

    // Methods
    void scan_callback(const LaserScan::SharedPtr msg) {
        auto request = std::make_shared<GetDirection::Request>();
        request->laser_data = *msg;
        auto future_result = client_->async_send_request(
            request,
            std::bind(&DirectionClient::handle_response, this, _1));
    }

    void handle_response(const rclcpp::Client<GetDirection>::SharedFuture future) {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Got response: %s", response->direction.c_str());
    }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionClient>());
  rclcpp::shutdown();
  return 0;
}