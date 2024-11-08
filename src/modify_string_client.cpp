/**
 * @file modify_string_client.cpp
 * @author koustubh (koustubh@umd.edu)
 * @brief A cpp file to run a client to call modify_string service
 * @version 2.0
 * @date 2024-11-08
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <chrono>
#include <iostream>
#include <rclcpp/client.hpp>
#include <string>

#include "beginner_tutorials/srv/modify_string.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Class to define a Service client object that calls the modify strings
 * object serve
 */
class ServiceClient : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Service Client object
   * @param node_name Name of the server client node
   */
  ServiceClient() : Node("server_client") {
    client_ = this->create_client<beginner_tutorials::srv::ModifyString>(
        "modify_string");
  }
  rclcpp::Client<beginner_tutorials::srv::ModifyString>::SharedPtr client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // check if the argument is provided while running the node
  if (argc < 2) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclpcpp"), "Usage: pass a string");
  }

  // create ServiceClient object pointer
  auto client = std::make_shared<ServiceClient>();

  // create pointer to handle requests
  auto request =
      std::make_shared<beginner_tutorials::srv::ModifyString::Request>();
  request->input = argv[1];

  // Loop to check if the service is available
  while (!client->client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR_STREAM(
          rclcpp::get_logger("rclcpp"),
          "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"),
                       "service not available, waiting again...");
  }

  // Send request to client server
  auto result = client->client_->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(client, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("rclcpp"),
        "RESPONSE: string set to: " << result.get()->output.c_str());
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "SERVICE CALL FAILED");
  }

  rclcpp::shutdown();
  return 0;
}
