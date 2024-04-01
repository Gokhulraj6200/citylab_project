#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/utilities.hpp"
#include <cstddef>
#include <custominterface/srv/get_direction.hpp>
#include <functional>
#include <memory>
#include <cstdlib>
#include <stdexcept>
#include <string>

using GetDirection = custominterface::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_server_node") {

    srv_ = create_service<GetDirection>(
        "/direction_service",
        std::bind(&DirectionService::handle_service, this, _1, _2));
  }
private:
    
   rclcpp::Service<GetDirection>::SharedPtr srv_;

   void handle_service(const std::shared_ptr<GetDirection::Request> request, 
                             std::shared_ptr<GetDirection::Response> response) {

    float total_dist_sec_right = 0;
    float total_dist_sec_front = 0;
    float total_dist_sec_left = 0;

    for (size_t i = 0; i < request->laser_data.ranges.size(); ++i) {
      float angle = request->laser_data.angle_min +
                    i * request->laser_data.angle_increment;

      if (angle <= 1.57 && angle >= 0.524) {
        total_dist_sec_left += request->laser_data.ranges[i];
      } else if (angle < 0.524 && angle >= -0.524) {
        total_dist_sec_front += request->laser_data.ranges[i];
      } else if (angle < -0.524 && angle >= -1.57) {
        total_dist_sec_right += request->laser_data.ranges[i];
      }
    }

    if (total_dist_sec_front > total_dist_sec_left &&
        total_dist_sec_front > total_dist_sec_right) {

      response->direction = "move forward";
    } else if (total_dist_sec_right > total_dist_sec_left &&
               total_dist_sec_right > total_dist_sec_front) {

      response->direction = "turn right";
    } else {
      response->direction = "turn right";
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}