/**
 * @file publisher_member_function.cpp
 * @author koustubh (koustubh@umd.edu)
 * @brief This file contains a simple ROS2 cpp publisher node
 * @version 0.2
 * @date 2024-11-04
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" 
#include "beginner_tutorials/srv/modify_string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    this->declare_parameter("publisher_rate", 500);
    publisher_rate_ = std::chrono::milliseconds(this->get_parameter("publisher_rate").as_int());
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(publisher_rate_, std::bind(&MinimalPublisher::timer_callback, this));
    service_ = this->create_service<beginner_tutorials::srv::ModifyString>(
        "modify_string", std::bind(&MinimalPublisher::changeString, this, std::placeholders::_1, std::placeholders::_2));
  }

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, from " + service_message + " : " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  void changeString(const std::shared_ptr<beginner_tutorials::srv::ModifyString::Request> request, 
                    const std::shared_ptr<beginner_tutorials::srv::ModifyString::Response> response){
    service_message = request->input;
    response->output = request->input;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request, Change string to: [%s]",request->input.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]", response->output.c_str());
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  std::string service_message = "Koustubh";
  rclcpp::Service<beginner_tutorials::srv::ModifyString>::SharedPtr service_;
  std::chrono::milliseconds publisher_rate_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
