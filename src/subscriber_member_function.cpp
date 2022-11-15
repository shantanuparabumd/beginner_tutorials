/**
 * @file subscriber_member_function.cpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @brief Simple Subscriber Template ROS2
 * @version 0.1
 * @date 2022-10-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
/**
 * @brief Class for Minimal subscriber 
 * 
 */

class MinimalSubscriber : public rclcpp::Node {
 public:
 /**
  * @brief Construct a new Minimal Subscriber object
  * 
  */
  MinimalSubscriber()
  : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
 /**
  * @brief Callback to process the message fetched from the topic
  * 
  * @param msg 
  */
  void topic_callback(const std_msgs::msg::String & msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
