/**
 * @file subscriber_member_function.cpp
 * @author Koustubh (koustubh@umd.edu)
 * @brief A simple cpp ROS2 subscriber
 * @version 2.0
 * @date 2024-11-08
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/timer.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/**
 * @brief Class MinimalSubscriber to define a node that publishes a string to a
 * topic
 *
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Subscriber object
   * @param node_name Name of the subscriber node
   */
  MinimalSubscriber() : Node("minimal_subscriber") {
    // create subscription handle
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    message_received_ = false;
    // create wall timer to check regularly for messages
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MinimalSubscriber::check_for_messages, this));
  }

 private:
  /**
   * @brief Topic callback to handle messages
   *
   * @param msg Message received from topic
   */
  void topic_callback(const std_msgs::msg::String& msg) {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg.data.c_str());
    message_received_ = true;
  }

  /**
   * @brief Function to check if the publisher has published yet
   *
   */
  void check_for_messages() {
    if (!message_received_) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Topic 'topic' not published yet..");
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool message_received_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
