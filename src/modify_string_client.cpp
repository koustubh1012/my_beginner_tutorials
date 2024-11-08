/**
 * @file modify_string_client.cpp
 * @author koustubh (koustubh@umd.edu)
 * @brief A cpp file to run a client to call modify_string service
 * @version 2.0
 * @date 2024-11-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <chrono>
#include <rclcpp/client.hpp>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "beginner_tutorials/srv/modify_string.hpp"

class ServiceClient : public rclcpp::Node {
    public:
        ServiceClient() : Node("server_client") {
            client_ = this->create_client<beginner_tutorials::srv::ModifyString>("modify_string");
        }
        rclcpp::Client<beginner_tutorials::srv::ModifyString>::SharedPtr client_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    if(argc<2){
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclpcpp"), "Usage: pass a string");
    }
    auto client = std::make_shared<ServiceClient>();

    auto request = std::make_shared<beginner_tutorials::srv::ModifyString::Request>();
    request->input = argv[1];

    while(!client->client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->client_->async_send_request(request);
  // Wait for the result.

    if (rclcpp::spin_until_future_complete(client, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "RESPONSE: string set to: " << result.get()->output.c_str());
    } else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "SERVICE CALL FAILED");
    }

    rclcpp::shutdown();
    return 0;
}
