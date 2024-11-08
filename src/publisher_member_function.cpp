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

// using std::placeholders::_1;
using STRING = std_msgs::msg::String;
using PUBLISHER = rclcpp::Publisher<STRING>::SharedPtr;
using TIMER = rclcpp::TimerBase::SharedPtr;
using PARAMETER_EVENT = std::shared_ptr<rclcpp::ParameterEventHandler>;
using PARAMETER_HANDLE = std::shared_ptr<rclcpp::ParameterCallbackHandle>;

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    // Set up the "freq" parameter with a default value of 2 Hz and a description
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
    param_desc.description = "Set callback frequency.";
    this->declare_parameter("freq", 2.0, param_desc);
    
    // Retrieve the initial frequency and set up the parameter event handler
    auto freq = this->get_parameter("freq").as_double();
    m_param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto paramCallbackPtr = std::bind(&MinimalPublisher::param_callback, this, std::placeholders::_1);
    m_param_handle_ = m_param_subscriber_->add_parameter_callback("freq", paramCallbackPtr);

    // Set up the publisher and timer with the initial frequency
    publisher_ = this->create_publisher<STRING>("topic", 10);
    auto period = std::chrono::milliseconds(static_cast<int>(1000 / freq));
    auto topicCallbackPtr = std::bind(&MinimalPublisher::timer_callback, this);
    timer_ = this->create_wall_timer(period, topicCallbackPtr);

    // Service to change string content dynamically
    service_ = this->create_service<beginner_tutorials::srv::ModifyString>(
        "modify_string", std::bind(&MinimalPublisher::changeString, this, std::placeholders::_1, std::placeholders::_2));
  }

 private:
  // Timer callback to publish messages at the specified frequency
  void timer_callback() {
    auto message = STRING();
    message.data = "Hello, from " + service_message + " : " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  // Service callback to change the string content
  void changeString(const std::shared_ptr<beginner_tutorials::srv::ModifyString::Request> request, 
                    const std::shared_ptr<beginner_tutorials::srv::ModifyString::Response> response) {
    service_message = request->input;
    response->output = request->input;
    RCLCPP_INFO(this->get_logger(), "Incoming request, Change string to: [%s]", request->input.c_str());
    RCLCPP_INFO(this->get_logger(), "Sending back response: [%s]", response->output.c_str());
  }

  // Callback to handle frequency parameter updates dynamically
  void param_callback(const rclcpp::Parameter & param) {
    RCLCPP_INFO(this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s: %.2f",
                param.get_name().c_str(), param.get_type_name().c_str(), param.as_double());

    // Update the timer with the new frequency
    auto period = std::chrono::milliseconds(static_cast<int>(1000 / param.as_double()));
    timer_->cancel(); // Stop the current timer
    timer_ = this->create_wall_timer(period, std::bind(&MinimalPublisher::timer_callback, this)); // Restart timer with new rate
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  std::string service_message = "Koustubh";
  rclcpp::Service<beginner_tutorials::srv::ModifyString>::SharedPtr service_;
  PARAMETER_EVENT m_param_subscriber_;
  PARAMETER_HANDLE m_param_handle_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

