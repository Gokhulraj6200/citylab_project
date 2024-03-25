#include <chrono>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("Patrol_node") {

    timer_ = this->create_wall_timer(
            500ms, std::bind(&Patrol::timer_callback, this));
}

private:
 rclcpp::TimerBase::SharedPtr timer_;
 void timer_callback() {}
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}
