#include "rclcpp/rclcpp.hpp"
#include "beginner_tutorials/srv/modify_string.hpp"
#include <cstdlib>
#include <iterator>
#include <string>
#include <vector>
#include <memory>

void add(const std::shared_ptr<beginner_tutorials::srv::ModifyString::Request> request,
          std::shared_ptr<beginner_tutorials::srv::ModifyString::Response>      response)
{
  std::vector<std::string> words{"Cat","Dog","Rat","Bat","Bird","Duck","Fox","Cow","Pig","Hen"};
  response->c = request->a +" "+ request->b+" "+words[rand()%10];

  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
  //               request->a, request->b);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("modify_string_server");

  rclcpp::Service<beginner_tutorials::srv::ModifyString>::SharedPtr service =
  node->create_service<beginner_tutorials::srv::ModifyString>("modify_string", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to modify string.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}