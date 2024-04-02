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
   // for simulation
    void handle_service(const std::shared_ptr<GetDirection::Request> request, 
                              std::shared_ptr<GetDirection::Response> response) 
    {
        float total_dist_sec_right = 0;
        float total_dist_sec_front = 0;
        float total_dist_sec_left = 0;
        size_t index = request->laser_data.ranges.size() * 0.75;
        float range_max = request->laser_data.range_max;
        for (size_t i = 0; i < request->laser_data.ranges.size()/2; ++i) {
            if ((i < request->laser_data.ranges.size()/6 ) && (request->laser_data.ranges[index] <= range_max)) {
                total_dist_sec_right += request->laser_data.ranges[index];
            }
            else if ((i < request->laser_data.ranges.size() / 3) && (request->laser_data.ranges[index] <= range_max)) {
                total_dist_sec_front += request->laser_data.ranges[index];
            }
            else if ((i >= request->laser_data.ranges.size()/ 3) && (request->laser_data.ranges[index] <= range_max)) {
                total_dist_sec_left += request->laser_data.ranges[index];
            }
            index++;
            if (index >= request->laser_data.ranges.size()) 
            {
              index = 0;
              }
        }
        if ((total_dist_sec_right > total_dist_sec_front) && (total_dist_sec_right > total_dist_sec_left)) {
            response->direction = "turn right";
        }
        else {
            response->direction = total_dist_sec_front > total_dist_sec_left ? "move forward" : "turn left";
        }
    }

   // for real robot
//    void handle_service(const std::shared_ptr<GetDirection::Request> request, 
//                              std::shared_ptr<GetDirection::Response> response) {

//     float total_dist_sec_right = 0;
//     float total_dist_sec_front = 0;
//     float total_dist_sec_left = 0;
//     int size = static_cast<int>(request->laser_data.ranges.size());

//     for (size_t i = 0; i < request->laser_data.ranges.size(); ++i) 
//     {
//       float angle = request->laser_data.angle_min +
//                     i * request->laser_data.angle_increment;

//       if (angle <= 1.57 && angle >= 0.524) {
//         total_dist_sec_left += request->laser_data.ranges[i];
//       } else if (angle < 0.524 && angle >= -0.524) {
//         total_dist_sec_front += request->laser_data.ranges[i];
//       } else if (angle < -0.524 && angle >= -1.57) {
//         total_dist_sec_right += request->laser_data.ranges[i];
//       }
//     }
//     if (request->laser_data.ranges[size/2] > 0.65) {
//             //RCLCPP_INFO(this->get_logger(), "Front object detected at %.2f m, moving forward.", front_reading_);
//             response->direction = "move forward";
//         }
//     else 
//     {
//     if (total_dist_sec_front > total_dist_sec_left &&
//         total_dist_sec_front > total_dist_sec_right) {

//       response->direction = "move forward";
//      } else if (total_dist_sec_right > total_dist_sec_left &&
//                total_dist_sec_right > total_dist_sec_front) {

//       response->direction = "turn right";
//       } else {
//       response->direction = "turn left";
//       }
//     }
//     }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}