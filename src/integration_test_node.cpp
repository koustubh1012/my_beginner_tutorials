#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

#include "beginner_tutorials/srv/modify_string.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std_msgs::msg::String;


////////////////////////////////////////////////
// Define Fixture
////////////////////////////////////////////////
auto Logger = rclcpp::get_logger (""); // create an initial Logger

class MyTestsFixture {
public:
  MyTestsFixture () 
  {
    
    //Create the node that performs the test. (aka Integration test node):
    testerNode = rclcpp::Node::make_shared ("IntegrationTestNode1");
    Logger = testerNode->get_logger(); // make sure message will appear in rqt_console

    
    //Declare a parameter for the duration of the test:
    testerNode->declare_parameter<double> ("test_duration");

    
     //Get the test duration value:
    TEST_DURATION = testerNode->get_parameter("test_duration").get_parameter_value().get<double>();
    RCLCPP_INFO_STREAM (Logger, "Got test_duration =" << TEST_DURATION);
  }

  ~MyTestsFixture ()
  {
  }

protected:
  double TEST_DURATION;
  rclcpp::Node::SharedPtr testerNode;
};

////////////////////////////////////////////////
// Test Case 1
////////////////////////////////////////////////

// In this test case, we will test if the talker node has started the
// modify_String service.
TEST_CASE_METHOD (MyTestsFixture, "test service server", "[service]") {

  // Now, create a client for the specific service we're looking for:
  auto client = testerNode->create_client<beginner_tutorials::srv::ModifyString>("modify_string");
  RCLCPP_INFO_STREAM (Logger, "myServiceName client created");

  // Get time from system
  rclcpp::Time start_time    = rclcpp::Clock().now(); // reads /clock, if "use_sim_time" is true
  bool service_found         = false;
  // Create duration variable
  rclcpp::Duration duration  = 0s;
  RCLCPP_INFO_STREAM (Logger, "Performing Test...");
  auto timeout = std::chrono::milliseconds ((int) (TEST_DURATION * 1000));
  // Check if service is available
  if (client->wait_for_service (timeout)) { // blocking 
    duration = (rclcpp::Clock().now() - start_time);
    service_found = true;
  }
  // Log the duration
  RCLCPP_INFO_STREAM (Logger, "duration = " << duration.seconds() << " service_found=" << service_found);
  // Test assertions - check that the service was found
  CHECK (service_found); 
}


////////////////////////////////////////////////
// Test Case 2
////////////////////////////////////////////////

/* In this test case, the node we will check the talker node has 
   created a topic and will check the string published by it*/

TEST_CASE_METHOD (MyTestsFixture, "test topic talker", "[topic]") {

  bool got_topic = false;

  // Define a callback that captures the additional parameter
  struct ListenerCallback {
    ListenerCallback(bool &gotTopic) : gotTopic_(gotTopic)
    {}
    void operator()(const String msg) const {
      // Log published message
      RCLCPP_INFO_STREAM (Logger, "I heard:" << msg.data.c_str());
      gotTopic_ = true;
      // Check if the string is published
      CHECK (msg.data.find("Hello, from") != std::string::npos);
    }
    bool &gotTopic_;
  };

  // Create subscriber for topic
  auto subscriber = testerNode->create_subscription<String> ("topic", 10, ListenerCallback (got_topic));
  rclcpp::Rate rate(10.0);       // 10hz checks
  auto start_time = rclcpp::Clock().now();
  auto duration   = rclcpp::Clock().now() - start_time;
  auto timeout    = rclcpp::Duration::from_seconds (TEST_DURATION);
  RCLCPP_INFO_STREAM (Logger, "duration = " << duration.seconds() << " timeout=" << timeout.seconds());
  while (!got_topic && (duration < timeout))
    {
      rclcpp::spin_some (testerNode);
      rate.sleep();
      duration = (rclcpp::Clock().now() - start_time);
    }
  RCLCPP_INFO_STREAM (Logger, "duration = " << duration.seconds() << " got_topic=" << got_topic);
  // Test assertions - check that the topic was received
  CHECK (got_topic);
}
