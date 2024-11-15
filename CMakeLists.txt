cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(catch_ros2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosbag2 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ModifyString.srv"
)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker 
  rclcpp
  std_msgs
  rclcpp
  rosidl_default_generators
  geometry_msgs
  rclcpp
  tf2_ros)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(talker ${cpp_typesupport_target})

add_executable(client src/modify_string_client.cpp)
ament_target_dependencies(client 
  rclcpp 
  std_msgs 
  rclcpp 
  rosidl_default_generators
)

target_link_libraries(client ${cpp_typesupport_target})

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener
  rclcpp
  std_msgs
)

add_executable(integration_test_node
  src/integration_test_node.cpp
)

target_link_libraries(integration_test_node ${cpp_typesupport_target})
target_link_libraries(integration_test_node
  catch_ros2::catch_ros2_with_node_main
)

ament_target_dependencies(integration_test_node
  rclcpp
  std_srvs
  std_msgs
  geometry_msgs
  tf2_ros
  rosidl_default_generators
)

catch_ros2_add_integration_test (ExampleIntegration_TestYAML
  LAUNCH_FILE integration_test.launch.yaml
)

install(TARGETS
  talker
  listener
  client
  integration_test_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
