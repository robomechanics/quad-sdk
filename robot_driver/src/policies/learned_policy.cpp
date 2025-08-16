#include "robot_driver/policies/learned_policy.hpp"

LearnedPolicy::LearnedPolicy(rclcpp::Node::SharedPtr node, std::string& robot_ns): node_(node), robot_ns_(robot_ns_){
}

void LearnedPolicy::init(double kp, double kd){

}

void LearnedPolicy::computeObservations(){

}

void LearnedPolicy::runInference(){
    
}

void LearnedPolicy::loadONNXModel(){

}

void LearnedPolicy::adjustPositionalTargets(){

}

bool LearnedPolicy::computeLegCommandArray(
    const quad_msgs::msg::RobotState &robot_state_msg,
    quad_msgs::msg::LegCommandArray &leg_command_array_msg        )