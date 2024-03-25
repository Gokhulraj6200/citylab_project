#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;
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
 
 }

private:
    
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        int size = static_cast<int>(msg->ranges.size());
        float front = msg->ranges[330];
        float left = msg->ranges[165];
        float right = msg->ranges[495];
        RCLCPP_INFO(this->get_logger(), "The laser scans array has a size of: %i", size);
        RCLCPP_INFO(this->get_logger(), "The front laser is: %.2f", front);
        RCLCPP_INFO(this->get_logger(), "The left laser is: %.2f", left);
        RCLCPP_INFO(this->get_logger(), "The right laser is: %.2f", right);
            auto velocity = geometry_msgs::msg::Twist();
            if (front > stop) {
                RCLCPP_INFO(this->get_logger(), "robot moves forward");
                velocity.linear.x = -0.1;
                publisher_->publish(velocity);
                
            }
            else {
                RCLCPP_INFO(this->get_logger(), "robot rotates");
                velocity.linear.x = 0;
                velocity.angular.z = 0.5;
                publisher_->publish(velocity);
            }


     }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  float stop= 0.7;

  void timer_callback() {}
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}