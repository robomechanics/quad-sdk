#include "robot_driver/NewPlatformTest.h"

NewPlatformTest::NewPlatformTest(ros::NodeHandle nh, int argc, char **argv) {
    nh_ = nh;
    argc_ = argc;
    argv_ = argv;

    // Load single joint command topic
    std::single_joint_cmd_topic;

    quad_utils::loadROSParamDefault(nh_, "robot_type", robot_name,
                                    std::string("new-platform"));
    
    quad_utils::loadROSParam(nh_, "topics/joint_1_command",
                            joint_1_cmd_topic);
    quad_utils::loadROSParam(nh_, "topics/joint_2_command",
                            joint_2_cmd_topic);
    quad_utils::loadROSParam(nh_, "topics/joint_3_command",
                            joint_3_cmd_topic);
    quad_utils::loadROSParam(nh_, "topics/joint_4_command",
                            joint_4_cmd_topic);
    quad_utils::loadROSParam(nh_, "topics/joint_5_command",
                            joint_5_cmd_topic);
    quad_utils::loadROSParam(nh_, "topics/joint_6_command",
                            joint_6_cmd_topic);
    
    quad_utils::loadROSParamDefault(nh_, "robot_driver/is_hardware", is_hardware_,
                                    true);

    quad_utils::loadROSParam(nh_, "/robot_driver/update_rate", update_rate_);
    quad_utils::loadROSParam(nh_, "/robot_driver/publish_rate", publish_rate_);

    quad_utils::loadROSParam(nh_, "/robot_driver/heartbeat_timeout",
                            heartbeat_timeout_);

    quad_utils::loadROSParam(nh_, "robot_driver/torque_limit", torque_limits_);

    // Setup pubs and subs
    single_joint_cmd_sub_ =
        nh_.subscribe(single_joint_cmd_topic, 1,
                        &RobotDriver::singleJointCommandCallback, this);
    remote_heartbeat_sub_ = nh_.subscribe(
        remote_heartbeat_topic, 1, &RobotDriver::remoteHeartbeatCallback, this);
    single_joint_cmd_pub_ = nh_.advertise<quad_msgs::MotorCommand>(single_joint_cmd_topic, 1);
    leg_command_array_pub_ =
        nh_.advertise<quad_msgs::LegCommandArray>(leg_command_array_topic, 1);
    robot_heartbeat_pub_ =
        nh_.advertise<std_msgs::Header>(robot_heartbeat_topic, 1);

    // Initialize hardware interface
    if (is_hardware_) {
        if (robot_name == "spirit") {
        hardware_interface_ = std::make_shared<SpiritInterface>();
        } else if (robot_name == "new-platform") {
            hardware_interface_ = std::make_shared<NewPlatformInterface>();
        } else {
        ROS_ERROR_STREAM("Invalid robot name " << robot_name
                                                << ", returning nullptr");
        hardware_interface_ = nullptr;
        }
    }


}

void NewPlatformTest::spin() {
  // Initialize timing params
  ros::Rate r(update_rate_);

  // Start the mblink connection
  if (is_hardware_) {
    hardware_interface_->loadInterface(argc_, argv_);
  }

  while (ros::ok()) {
    // Collect new messages on subscriber topics and publish heartbeat
    ros::spinOnce();

    // Get the newest state information
    updateState();

    // Compute the leg command and publish if valid
    bool is_valid = updateControl();
    publishControl(is_valid);

    // // Publish state and heartbeat
    publishState();
    publishHeartbeat();

    // Enforce update rate
    r.sleep();
  }

  // Close the mblink connection
  if (is_hardware_) {
    hardware_interface_->unloadInterface();
  }
}
