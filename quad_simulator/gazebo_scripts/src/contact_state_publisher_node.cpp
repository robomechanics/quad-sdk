#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "gazebo_scripts/contact_state_publisher.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Create the Node
  //   auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<rclcpp::Node>("contact_state_publisher_node");
  auto contact_state_publisher = std::make_shared<ContactStatePublisher>(node);

  // Create Contact State Publisher with node pointer
  contact_state_publisher->spin();
  rclcpp::shutdown();
  return 0;
}
