/**
 * @file publisher_member_function.cpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @brief Template for Publisuher ROS2
 * @version 0.1
 * @date 2022-10-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/modify_string.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher()  //Object for node is created using a constructor
  : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    client = this->create_client<beginner_tutorials::srv::ModifyString>("modify_string");
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          exit (EXIT_FAILURE);
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }
  }

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, " +Message + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "808X ROS2: '%s'", message.data.c_str());
    publisher_->publish(message);
    if(count_%10==0){
      call_service();
    }
  }
  int call_service(){
    auto request = std::make_shared<beginner_tutorials::srv::ModifyString::Request>();
    request->a = "Random";
    request->b = "Animal";
    RCLCPP_INFO (this->get_logger(), "Callin Service to Modify string");
    auto callbackPtr  = std::bind(&MinimalPublisher::response_callback, this, _1);
    client->async_send_request(request, callbackPtr);
    return 1;
  }
  void response_callback (rclcpp::Client<beginner_tutorials::srv::ModifyString>::SharedFuture future) {
    // Process the response
    RCLCPP_INFO (this->get_logger(), "Got String: %s", future.get()->c.c_str());
    Message=future.get()->c.c_str();
  }
  std::string Message="Shantanu";
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Client<beginner_tutorials::srv::ModifyString>::SharedPtr client;
  size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
