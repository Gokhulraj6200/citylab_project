#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>

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
    
    //pi
    pi_= std::atan2(0 , -1);
 
 }

private:
    
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
        int max_index = 0; 
        float max_obj = 0;
        int size = static_cast<int>(msg->ranges.size());
        float front = msg->ranges[size * 0.5];
        float left = msg->ranges[size * 0.25];
        float right = msg->ranges[size * 0.75];
        float max_range = msg->range_max;
        float edge_val = max_range;
        float direction_;
        RCLCPP_INFO(this->get_logger(), "The laser scans array has a size of: %i", size);
        RCLCPP_INFO(this->get_logger(), "The front laser is: %.2f", front);
        RCLCPP_INFO(this->get_logger(), "The left laser is: %.2f", left);
        RCLCPP_INFO(this->get_logger(), "The right laser is: %.2f", right);

        for(int i = 0; i < size; i++){
                if (msg->ranges[i] < max_range && msg->ranges[i] > max_obj)
                {
                    max_obj = msg->ranges[i];
                    max_index = i < 0.25 * size ? i : i - size;
                }
                /*Introducing edge values to avoid collision*/
                if (msg->ranges[i] < edge_val && (i < size * 0.125 || i > size * 0.875 )) {
                    edge_val = msg->ranges[i];
                    deviation_ = i < 0.25 * size ? -1 : 1;
                }
        }
        direction_ = max_index * 2 * pi_ / size;
        RCLCPP_INFO(this->get_logger(), "The max object is: %.2f and index is: %i", max_obj , max_index);
        auto velocity = geometry_msgs::msg::Twist();
        /* code logic for simulation*/
        
        if (edge_val > 0.4 && msg->ranges[0] > 0.8) {
            RCLCPP_INFO(this->get_logger(), "robot moves forward");
            velocity.linear.x = 0.1;
            velocity.angular.z = 0.0;
            publisher_->publish(velocity);
        }
        else if (edge_val > 0.4 && msg->ranges[0] < 0.8){
            RCLCPP_INFO(this->get_logger(), "robot rotates");
            velocity.linear.x = 0.05;
            velocity.angular.z = direction_;
            publisher_->publish(velocity);
        }
        else {
            RCLCPP_INFO(this->get_logger(), "robot turns slightly");
            velocity.linear.x = 0.0;
            velocity.angular.z = deviation_ * 0.5;
            publisher_->publish(velocity);
        }
            
           /* code logic for real robot */
           // if (front >= 0.65) {
           //     if(right <0.25 ){ velocity.angular.z = -0.05;
           //     publisher_->publish(velocity);}
           //     else if(left <0.25){ velocity.angular.z = 0.05;
           //     publisher_->publish(velocity);}
           //     RCLCPP_INFO(this->get_logger(), "robot moves forward");
           //     velocity.linear.x = 0.1;
           //     publisher_->publish(velocity);
                
           // }
           // else {

           // if (max_index > size * 0.5){
           //     RCLCPP_INFO(this->get_logger(), "robot rotates counterclockwise");
           //     velocity.linear.x = 0;
           //     velocity.angular.z = direction_/2;
           //     publisher_->publish(velocity);
           // }
           // else 
           // {
           //     RCLCPP_INFO(this->get_logger(), "robot rotates clockwise");
           //     velocity.linear.x = 0;
           //     velocity.angular.z = -direction_/2;
           //     publisher_->publish(velocity);
           // }
            
           // }
            

     }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  //float stop= 0.7;
  float pi_;
  float deviation_;

  void timer_callback() {}
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}