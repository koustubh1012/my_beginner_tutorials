/**
 * @file publisher_member_function.cpp
 * @author koustubh (koustubh@umd.edu)
 * @brief This file contains a simple ROS2 cpp publisher node
 * @version 2.0
 * @date 2024-11-08
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <string>

#include "beginner_tutorials/srv/modify_string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

using STRING = std_msgs::msg::String;
using PUBLISHER = rclcpp::Publisher<STRING>::SharedPtr;
using TIMER = rclcpp::TimerBase::SharedPtr;
using PARAMETER_EVENT = std::shared_ptr<rclcpp::ParameterEventHandler>;
using PARAMETER_HANDLE = std::shared_ptr<rclcpp::ParameterCallbackHandle>;

/**
 * @brief Class MinimalPublisher to define a node that publishes a string to a
 * topic
 *
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Publisher object
   * @param node_name Name of the publisher node
   * @param freq ROS parameter to set the frequency of publisher
   */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    this->get_logger().set_level(rclcpp::Logger::Level::Debug);
    // Set up the "freq" parameter with a default value of 2 Hz and a
    // description
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
    param_desc.description = "Set callback frequency.";
    this->declare_parameter("freq", 2.0, param_desc);

    // Retrieve the initial frequency and set up the parameter event handler
    auto freq = this->get_parameter("freq").as_double();
    m_param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto paramCallbackPtr = std::bind(&MinimalPublisher::param_callback, this,
                                      std::placeholders::_1);
    m_param_handle_ =
        m_param_subscriber_->add_parameter_callback("freq", paramCallbackPtr);

    // Set up the publisher and timer with the initial frequency
    publisher_ = this->create_publisher<STRING>("topic", 10);
    auto period = std::chrono::milliseconds(static_cast<int>(1000 / freq));
    auto topicCallbackPtr = std::bind(&MinimalPublisher::timer_callback, this);
    timer_ = this->create_wall_timer(period, topicCallbackPtr);

    // Service to change string content dynamically
    service_ = this->create_service<beginner_tutorials::srv::ModifyString>(
        "modify_string",
        std::bind(&MinimalPublisher::changeString, this, std::placeholders::_1,
                  std::placeholders::_2));

    // Initialised tf broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

 private:
  /**
   * @brief Timer callback to publish messages at the specified frequency
   *
   */
  void timer_callback() {
    auto message = STRING();
    message.data =
        "Hello, from " + service_message + " : " + std::to_string(count_++);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Publishing: ' " << message.data.c_str() << "'");
    // Print publisher rate after every 10 iterations to debug
    if (count_ % 10 == 0) {
      RCLCPP_DEBUG_STREAM(this->get_logger(),
                          "Current publisher rate: "
                              << this->get_parameter("freq").as_double());
    }
    publisher_->publish(message);

    rclcpp::Time now = this->get_clock()->now();

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "world";
    t.child_frame_id = "talker";
    t.transform.translation.x = 1.0;
    t.transform.translation.y = 2.0;
    t.transform.translation.z = 3.0;
    t.transform.rotation.x = 0.707;
    t.transform.rotation.y = 0.707;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 0.0;

    tf_broadcaster_->sendTransform(t);
  }

  /**
   * @brief Service callback to change the string content
   *
   * @param request request sent to server
   * @param response response sent by server
   */
  void changeString(
      const std::shared_ptr<beginner_tutorials::srv::ModifyString::Request>
          request,
      const std::shared_ptr<beginner_tutorials::srv::ModifyString::Response>
          response) {
    service_message = request->input;
    response->output = request->input;
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Incoming request, Change string to: ["
                           << request->input.c_str() << "]");
    RCLCPP_INFO_STREAM(this->get_logger(), "Sending back response: ["
                                               << response->output.c_str()
                                               << "]");
  }

  /**
   * @brief Callback to handle frequency parameter updates dynamically
   *
   * @param param frequency change parameter
   */
  void param_callback(const rclcpp::Parameter& param) {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "cb: Updated Frequency To: " << param.as_double());

    if (param.as_double() > 1000.0) {
      RCLCPP_WARN_STREAM(
          this->get_logger(),
          "Frequency too high, might not able to publish at same rate");
    }

    // Update the timer with the new frequency
    auto period =
        std::chrono::milliseconds(static_cast<int>(1000 / param.as_double()));
    timer_->cancel();  // Stop the current timer
    timer_ = this->create_wall_timer(
        period, std::bind(&MinimalPublisher::timer_callback,
                          this));  // Restart timer with new rate
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  std::string service_message = "Koustubh";
  rclcpp::Service<beginner_tutorials::srv::ModifyString>::SharedPtr service_;
  PARAMETER_EVENT m_param_subscriber_;
  PARAMETER_HANDLE m_param_handle_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
