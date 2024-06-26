#include "custominterface/action/go_to_pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <functional>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

class GoToPose : public rclcpp::Node {
public:
  using actionpose = custominterface::action::GoToPose;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<actionpose>;

  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("my_action_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<actionpose>(
        this, "go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    odom_subscription = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&GoToPose::odomCallback, this, _1));

  }

private:
  rclcpp_action::Server<actionpose>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;
  geometry_msgs::msg::Pose2D desired_pos_ = geometry_msgs::msg::Pose2D();
  geometry_msgs::msg::Pose2D current_pos_ = geometry_msgs::msg::Pose2D();

  double Kp_ = 0.50;
  double Ki_ = 0.01;
  double Kd_ = 0.1;
  double prev_error_ = 0;
  double int_error_ = 0;

  double linear_maxspeed_ = 0.2;
  double angular_maxspeed_ = 0.5;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const actionpose::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal with: x=%f, y=%f, theta=%f",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);

    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Cancelling goal");

    geometry_msgs::msg::Twist stopbot;
    stopbot.linear.x = 0.0;
    stopbot.angular.z = 0.0;
    publisher_->publish(stopbot);
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<actionpose::Result>();
    auto move = geometry_msgs::msg::Twist();
    auto feedback = std::make_shared<actionpose::Feedback>();
    auto &feedback_data = feedback->current_pos;

    rclcpp::Rate loop_rate(10);
    double tolerance = 0.05;

    while (rclcpp::ok()) {
      double current_x = current_pos_.x;
      double current_y = current_pos_.y;

      double desired_x = goal->goal_pos.x;
      double desired_y = goal->goal_pos.y;

      feedback_data.x = current_x;
      feedback_data.y = current_y;
      feedback_data.theta = current_pos_.theta;

      double dx = desired_x - current_x;
      double dy = desired_y - current_y;

      double d_error = std::sqrt(std::pow(desired_x - current_x, 2) +
                                 std::pow(desired_y - current_y, 2));

      if (d_error <= tolerance) {
        result->status = "Linear Goal achieved.";
        move.linear.x = 0.0;

        while (rclcpp::ok()) {
          double angle_error = goal->goal_pos.theta * (M_PI / 180.0) - current_pos_.theta;

          RCLCPP_INFO(this->get_logger(), "Angle Error: %f", angle_error);
          RCLCPP_INFO(this->get_logger(), "act_theta: %f, req_theta: %f",
                      current_pos_.theta, goal->goal_pos.theta * (M_PI / 180.0));

          while (angle_error > M_PI) {
            angle_error -= 2.0 * M_PI;
          }
          while (angle_error < -M_PI) {
            angle_error += 2.0 * M_PI;
          }

          if (abs(angle_error) < 0.1) {
            move.linear.x = 0.0;
            move.angular.z = 0.0;
            publisher_->publish(move);
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal achieved");
            break;
          }

          double angular_speed2 = Kp_ * angle_error;
          angular_speed2 = std::max(
              -angular_maxspeed_, std::min(angular_speed2, angular_maxspeed_));
          move.linear.x = 0.0;
          move.angular.z = angular_speed2;
          publisher_->publish(move);
        }

        while (rclcpp::ok()) {
          loop_rate.sleep();
        }
      } else {
        double tan_inverse = atan2(dy, dx);
        double a_error = tan_inverse - current_pos_.theta;

        while (a_error > 3.145) {
          a_error -= 2 * 3.145;
        }
        while (a_error < -3.145) {
          a_error += 2 * 3.145;
        }

        double angular_speed =
            Kp_ * a_error + Ki_ * int_error_ + Kd_ * (a_error - prev_error_);
        prev_error_ = a_error;
        int_error_ += a_error;

        angular_speed = std::max(-angular_maxspeed_,
                                 std::min(angular_speed, angular_maxspeed_));

        double linear_speed =
            std::max(0.0, std::min(d_error * 0.1, linear_maxspeed_));
        move.linear.x = linear_speed;
        move.angular.z = angular_speed;

        publisher_->publish(move);
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(this->get_logger(), "Error: %f", d_error);
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, theta: %f", current_x,
                    current_y, current_pos_.theta);
      }

      loop_rate.sleep();
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;
    current_pos_.theta = yaw;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<GoToPose>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);

  std::thread executor_thread([&executor]() { executor.spin(); });
  executor_thread.join();

  return 0;
}