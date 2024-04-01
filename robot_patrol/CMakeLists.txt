cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custominterface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(robot_patrol REQUIRED)
find_package(custominterface REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Excecutables
add_executable(patrol src/patrol.cpp)
add_executable(direction_server_node src/direction_service.cpp)
add_executable(test_service_node src/test_service.cpp)

# Target dependencies
ament_target_dependencies(patrol rclcpp sensor_msgs geometry_msgs)
ament_target_dependencies(direction_server_node rclcpp robot_patrol custominterface)
ament_target_dependencies(test_service_node rclcpp robot_patrol sensor_msgs custominterface)

# Install targets
install(TARGETS patrol DESTINATION lib/${PROJECT_NAME})
install(TARGETS direction_server_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS test_service_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies
ament_export_dependencies(ament_cmake rclcpp sensor_msgs geometry_msgs) 

ament_package()