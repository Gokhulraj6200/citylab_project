#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include "custominterface/srv/get_direction.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>

using namespace std::chrono_literals;
using GetDirection = custominterface::srv::GetDirection;
using std::placeholders::_1;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Patrol::laserCallback, this, _1));

    timer_ = this->create_wall_timer(
            500ms, std::bind(&Patrol::timer_callback, this));
    
    //pi
    //pi_= std::atan2(0 , -1);
 
 }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("direction_client");
    rclcpp::Client<GetDirection>::SharedPtr client =
      node->create_client<GetDirection>("direction_service");
    float stop= 0.6;
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {

    RCLCPP_INFO(this->get_logger(), "starting the service");

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "service not available, waiting again...");
    }
    
    auto velocity = geometry_msgs::msg::Twist();
    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data.angle_min = msg->angle_min;
    request->laser_data.angle_max = msg->angle_max;
    request->laser_data.angle_increment = msg->angle_increment;
    request->laser_data.ranges = msg->ranges;
    //int size = static_cast<int>(msg->ranges.size());
 

    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto result = result_future.get();
    //   if (msg->ranges[size/2] > 0.65) {
    //         //RCLCPP_INFO(this->get_logger(), "Front object detected at %.2f m, moving forward.", front_reading_);
    //         velocity.linear.x = 0.1;
    //         velocity.angular.z = 0.0;
    //         publisher_->publish(velocity);
    //     }
      if (result->direction == "move forward") {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned forward");
        velocity.linear.x = 0.1;
        velocity.angular.z = 0.0;
        publisher_->publish(velocity);
      } else if (result->direction == "turn left") {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned left");
        velocity.linear.x = 0.1;
        velocity.angular.z = 0.5;
        publisher_->publish(velocity);
      } else if (result->direction == "turn right") {

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned right");
        velocity.linear.x = 0.1;
        velocity.angular.z = -0.5;
        publisher_->publish(velocity);
      }
     
    } 
    else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Failed to call service /moving");
    }
    }

  void timer_callback() {}
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}