#include <rclcpp/rclcpp.hpp>

#include <iostream>

#include "robot_driver/robot_driver.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("robot_driver_node");
    RobotDriver robot_driver(node, argc, argv);
    robot_driver.spin();
    rclcpp::shutdown();
    return 0;
}
