#include "force_applicator/force_applicator.hpp"
#include <iostream>

ForceApplicator::ForceApplicator(rclcpp::Node::SharedPtr node) : node_(node){

    std::string robot_state_topic, force_marker_topic, wrench_persist_topic, wrench_clear_topic;

    quad_utils::loadROSParam(node_, "world", world_name_);
    quad_utils::loadROSParam(node_, "robot_type", robot_type_);
    quad_utils::loadROSParam(node_, "robot_ns", robot_ns_);
    quad_utils::loadROSParam(node_, "mode", mode_);
    quad_utils::loadROSParam(node_, "force_mode", force_mode_);
    quad_utils::loadROSParam(node_, "link", link_);
    quad_utils::loadROSParam(node_, "force_applicator.fixed.force.x", force_x_);
    quad_utils::loadROSParam(node_, "force_applicator.fixed.force.y", force_y_);
    quad_utils::loadROSParam(node_, "force_applicator.fixed.force.z", force_z_);
    quad_utils::loadROSParam(node_, "force_applicator.fixed.torque.x", torque_x_);
    quad_utils::loadROSParam(node_, "force_applicator.fixed.torque.y", torque_y_);
    quad_utils::loadROSParam(node_, "force_applicator.fixed.torque.z", torque_z_);
    quad_utils::loadROSParam(node_, "force_applicator.duration.dt", dt_);
    quad_utils::loadROSParam(node_, "force_applicator.duration.update_rate", update_rate_);
    quad_utils::loadROSParam(node_, "force_applicator.random.force_mag_min", force_mag_min_);
    quad_utils::loadROSParam(node_, "force_applicator.random.force_mag_max", force_mag_max_);
    quad_utils::loadROSParam(node_, "force_applicator.random.torque_mag_min", torque_mag_min_);
    quad_utils::loadROSParam(node_, "force_applicator.random.torque_mag_max", torque_mag_max_);
    quad_utils::loadROSParam(node_, "force_applicator.periodic.period", period_);
    quad_utils::loadROSParam(node_, "force_applicator.distance.distance_threshold", distance_threshold_);

    // Load in ROS topics
    quad_utils::loadROSParam(node_, "force_applicator.topics.state.ground_truth", robot_state_topic);
    quad_utils::loadROSParam(node_, "force_applicator.topics.visualization.force_torque_markers", force_marker_topic);
    quad_utils::loadROSParam(node_, "force_applicator.topics.wrench.wrench_persist", wrench_persist_topic);
    quad_utils::loadROSParam(node_, "force_applicator.topics.wrench.wrench_clear", wrench_clear_topic);

    // Define Subscribers and Publishers
    robot_state_sub_ = node_->create_subscription<quad_msgs::msg::RobotState>(robot_state_topic, 10, 
        std::bind(&ForceApplicator::robotStateCallback, this, std::placeholders::_1));
    force_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(force_marker_topic, 10);
    wrench_persist_pub_ = node_->create_publisher<ros_gz_interfaces::msg::EntityWrench>(wrench_persist_topic, 10);
    wrench_clear_pub_ = node_->create_publisher<ros_gz_interfaces::msg::Entity>(wrench_clear_topic, 10);

    // Build GZ transport topics, Remove .sdf tag
    if (world_name_.size() >= 4 && world_name_.compare(world_name_.size()-4, 4, ".sdf") == 0){
        world_name_.erase(world_name_.size()-4);
    }
    wrench_topic_persist_ = "/world/" + world_name_ + "/wrench/persistent";
    wrench_topic_clear_   = "/world/" + world_name_ + "/wrench/clear";

    RCLCPP_INFO(node_->get_logger(), "ApplyLinkWrench using [%s] (start), clear [%s]",
                wrench_topic_persist_.c_str(), wrench_topic_clear_.c_str());

    // Seed the random number generator
    rng_.seed(std::random_device{}());
}

void ForceApplicator::robotStateCallback(const quad_msgs::msg::RobotState::SharedPtr msg){
    last_robot_state_msg_ = *msg;
    have_pose_ = true;
}

double ForceApplicator::computeEuclideanDistance(const Eigen::Vector3d& point1, 
                                            const Eigen::Vector3d& point2){
    return (point2 - point1).norm();
}

bool ForceApplicator::shouldTrigger(){
    if (mode_ == "periodic"){
        const auto now = node_->now();
        if (last_fire_time_.nanoseconds() == 0){
            last_fire_time_ = now;
            return false;
        }
        if (now - last_fire_time_ >= rclcpp::Duration::from_seconds(period_)){
            last_fire_time_ = now;
            return true;
        }
        return false;
    }
    else if (mode_ == "distance" && have_pose_){
        Eigen::Vector3d curr_robot_pose  = Eigen::Vector3d::Zero();
        curr_robot_pose << last_robot_state_msg_.body.pose.position.x, 
                            last_robot_state_msg_.body.pose.position.y,  
                             last_robot_state_msg_.body.pose.position.z;   
        if (computeEuclideanDistance(last_robot_pose_, curr_robot_pose) > distance_threshold_){
            last_robot_pose_ = curr_robot_pose;
            return true;
        }
        return false;
    }
    return false;
}

void ForceApplicator::updateMarker(){
    // Add Marker Construction Code Here
    last_robot_marker_msg_.header.frame_id = "robot_1_ground_truth/body";
    last_robot_marker_msg_.header.stamp = node_->now();
    last_robot_marker_msg_.ns = "apply_force";
    last_robot_marker_msg_.id = 0;
    last_robot_marker_msg_.type = visualization_msgs::msg::Marker::ARROW;
    last_robot_marker_msg_.action = visualization_msgs::msg::Marker::ADD;
    last_robot_marker_msg_.pose.position.x = 0.25;
    last_robot_marker_msg_.pose.position.y = -0.4;
    last_robot_marker_msg_.pose.position.z = 0;
    last_robot_marker_msg_.pose.orientation.w = 1.0;

    last_robot_marker_msg_.scale.x = 0.2;  // force_magnitude_*0.1;  // Arrow length
    last_robot_marker_msg_.scale.y = 0.02;  // Arrow width
    last_robot_marker_msg_.scale.z = 0.02;  // Arrow height


    last_robot_marker_msg_.color.r = 1.0;   
    last_robot_marker_msg_.color.g = 0.0;
    last_robot_marker_msg_.color.b = 0.0;
    last_robot_marker_msg_.color.a = 1.0;
    last_robot_marker_msg_.lifetime = rclcpp::Duration::from_seconds(0.75);


    // Modify this to compute the Orientation of the Force Applied
    if (force_magnitude_ > 0.0) {
        tf2::Quaternion q;
        q.setRPY(0, 0, std::atan2(fy, fx));
        q.normalize();
        last_robot_marker_msg_.pose.orientation = tf2::toMsg(q);
    } 
    else {
        last_robot_marker_msg_.pose.orientation = geometry_msgs::msg::Quaternion{};
        last_robot_marker_msg_.pose.orientation.w = 1.0;  // identity
    }

    force_marker_pub_->publish(last_robot_marker_msg_);
}

void ForceApplicator::applyForce(){
    // Determine force vector
    double tx, ty, tz;
    if (force_mode_ == "random") {
      std::uniform_real_distribution<double> f_mag_distribution(force_mag_min_, force_mag_max_);
      std::uniform_real_distribution<double> t_mag_distribution(torque_mag_min_, torque_mag_max_);
      fx = f_mag_distribution(rng_); fy = f_mag_distribution(rng_); fz = f_mag_distribution(rng_);
      tx = t_mag_distribution(rng_); ty = t_mag_distribution(rng_); tz = t_mag_distribution(rng_);
    } 
    else if (force_mode_ == "yaml") {
      // fixed magnitude along provided direction
      fx = force_x_; fy = force_y_; fz = force_z_;
      tx = torque_x_; ty = torque_y_; tz = torque_z_;
    }

    force_magnitude_ = Eigen::Vector3d(fx, fy, fz).norm();

    // Build and Publish ROS Entity Wrench
    ros_gz_interfaces::msg::EntityWrench ew;
    ew.header.stamp = node_->now();

    ew.entity.name = link_;
    ew.entity.type = ros_gz_interfaces::msg::Entity::LINK;

    ew.wrench.force.x = fx;
    ew.wrench.force.y = fy;
    ew.wrench.force.z = fz;

    ew.wrench.torque.x = tx;
    ew.wrench.torque.y = ty;
    ew.wrench.torque.z = tz;

    wrench_persist_pub_->publish(ew);

    // Arm 
    if (clear_timer_) {
        clear_timer_->cancel();
        clear_timer_->reset();
    }
    clear_timer_ = rclcpp::create_timer(node_, node_->get_clock(), 
        rclcpp::Duration::from_seconds(dt_),
        [this](){
            ros_gz_interfaces::msg::Entity e; 
            e.name= link_; 
            e.type = ros_gz_interfaces::msg::Entity::LINK;
            wrench_clear_pub_->publish(e);

            if (clear_timer_){
                clear_timer_->cancel();
                clear_timer_.reset();
            }
        });
    }

void ForceApplicator::spin() {
    // Initialize Timing Params
    rclcpp::Rate r(update_rate_, node_->get_clock());
    while(rclcpp::ok()){
        rclcpp::spin_some(node_);
        if (shouldTrigger()){
            // Get the Newest Force Information
            applyForce();
            updateMarker();
        }
        // Enforce Update Rate
        r.sleep();
    }
}