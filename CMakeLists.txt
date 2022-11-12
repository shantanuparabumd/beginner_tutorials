cmake_minimum_required(VERSION 3.5)
project(beginner_tutorials)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ModifyString.srv"  
)


#Talker Executable
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)


#Listener Executable
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)


add_executable(server src/modify_string_server.cpp)
ament_target_dependencies(server rclcpp std_msgs)

add_executable(client src/modify_string_client.cpp)
ament_target_dependencies(client rclcpp std_msgs)


install(TARGETS
  talker
  listener
  server
  client
  DESTINATION lib/${PROJECT_NAME})

rosidl_target_interfaces(server 
${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(client 
${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(talker 
${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)
ament_package()