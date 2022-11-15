/**
 * @file modify_string_server.cpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @brief Node to serve the response for a request
 * @version 0.1
 * @date 2022-11-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "beginner_tutorials/srv/modify_string.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdlib>
#include <iterator>
#include <string>
#include <vector>
#include <memory>
using ModString = beginner_tutorials::srv::ModifyString;

void add(const std::shared_ptr<ModString::Request> request,
          std::shared_ptr<ModString::Response> response) {
  std::vector<std::string> words{"Cat", "Dog", "Rat", "Bat", "Bird",
                "Duck", "Fox", "Cow", "Pig", "Hen"};
  response->c = request->a +" "+ request->b+" "+words[rand()%10];
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node =
        rclcpp::Node::make_shared("modify_string_server");

  rclcpp::Service<ModString>::SharedPtr service =
  node->create_service<ModString>("modify_string", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to modify string.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
