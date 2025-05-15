#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include "contact_state_publisher.h"

int main(int argc, char**argv)
{
    rclcpp::init(argc, argv)
    
    // Create the Node
    auto options= rclcpp::NodeOptions();
    auto node = std::make_shared<ContactStatePublisher>();

    // Create Contact State Publisher with node pointer
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}