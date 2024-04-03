#include <chrono>
#include "geometry_msgs/msg/twist.hpp"
#include <rclcpp/rclcpp.hpp>
#include "custominterface/srv/get_direction.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <string.h>

using namespace std::chrono_literals;
using GetDirection = custominterface::srv::GetDirection;
using std::placeholders::_1;

class Patrol : public rclcpp::Node {
public:
    Patrol() : rclcpp::Node("patrol_node") {

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            10, 
            std::bind(&Patrol::laserCallback, this, _1));
  
        client_ = this->create_client<GetDirection>("/direction_service");

        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service.");
        }
        
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Client<GetDirection>::SharedPtr client_;
    float front_laser_;

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto request = std::make_shared<GetDirection::Request>();
        request->laser_data = *msg;
        this->front_laser_ = msg->ranges[0];
        auto future_result = client_->async_send_request(
            request,
            std::bind(&Patrol::publishVel, this, _1));
    }

    void publishVel(const rclcpp::Client<GetDirection>::SharedFuture future) {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Got response: %s", response->direction.c_str());
        geometry_msgs::msg::Twist velocity;
        if (front_laser_ > 1.0) {
            RCLCPP_INFO(this->get_logger(), "safe to move front");
            velocity.linear.x = 0.1;
            velocity.angular.z = 0.0;
            publisher_->publish(velocity);
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Using service direction.");
            if (response->direction == "forward") {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned forward");
                velocity.linear.x = 0.1;
                velocity.angular.z = 0.0;
                publisher_->publish(velocity);
            }
            else if (response->direction == "left") {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned left");
                velocity.linear.x = 0.1;
                velocity.angular.z = 0.5;
                publisher_->publish(velocity);
            }
            else if (response->direction == "right") {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned right");
                velocity.linear.x = 0.1;
                velocity.angular.z = -0.5;
                publisher_->publish(velocity);
            }
            else {
                velocity.linear.x = 0.0;
                velocity.angular.z = 0.0;
                publisher_->publish(velocity);
            }
        }
        publisher_->publish(velocity);
    }

};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}