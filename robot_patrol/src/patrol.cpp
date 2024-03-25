#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Patrol::laserCallback, this, _1));

    timer_ = this->create_wall_timer(
            500ms, std::bind(&Patrol::timer_callback, this));
 
 }

private:
    
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        int size = static_cast<int>(msg->ranges.size());
        float front = msg->ranges[size/2];
        float start = msg->range_min;
        float end = msg->range_max;
        RCLCPP_INFO(this->get_logger(), "The laser scans array has a size of: %i", size);
        RCLCPP_INFO(this->get_logger(), "The front laser is: %.2f and index is %i", front, size/2);
        RCLCPP_INFO(this->get_logger(), "The laser scans start at: %.2f", start);
        RCLCPP_INFO(this->get_logger(), "The laser scans ends at: %.2f", end);
     }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

  void timer_callback() {}
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}
