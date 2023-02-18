#include <sensor_msgs/Joy.h>

#include "robot_driver/robot_driver.h"

using namespace std::chrono;

int numFeet = 1;
int currAxis = 0;
float joyPos = 0;

uint64_t getCurrTime() {
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

void sensorMsgCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    //Check for change in axis
    for (int i=0; i < 4; i++) {
        if (joy->buttons[i] == 1) {
            currAxis = i;
        }
    }

    //Check joystick position for motor command
    joyPos = joy->axes[1];
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "new_platform_test_node");
    ros::NodeHandle nh_;

    // Load rosparams from parameter server
    std::string robot_name, joint_state_topic, leg_command_array_topic,
        robot_heartbeat_topic, single_joint_cmd_topic, sensor_msg_topic;
    int motor_0_id, motor_1_id, motor_2_id;
    std::vector<double> motor_0_pos;
    std::vector<double> motor_1_pos;
    std::vector<double> motor_2_pos;

    quad_utils::loadROSParam(nh_, "topics/state/joints", joint_state_topic);
    quad_utils::loadROSParam(nh_, "topics/heartbeat/robot",
                            robot_heartbeat_topic);
    quad_utils::loadROSParam(nh_, "topics/control/joint_command",
                            leg_command_array_topic);
    quad_utils::loadROSParam(nh_, "topics/control/single_joint_command",
                            single_joint_cmd_topic);
    quad_utils::loadROSParam(nh_, "motor_0/id", motor_0_id);
    quad_utils::loadROSParam(nh_, "motor_1/id", motor_1_id);
    quad_utils::loadROSParam(nh_, "motor_2/id", motor_2_id);
    quad_utils::loadROSParam(nh_, "motor_0/position", motor_0_pos);
    quad_utils::loadROSParam(nh_, "motor_1/position", motor_1_pos);
    quad_utils::loadROSParam(nh_, "motor_2/position", motor_2_pos);

    std::vector<int> motor_ids = {motor_0_id, motor_1_id, motor_2_id};

    std::vector<std::vector<double>> motor_pos = {motor_0_pos, 
                                                    motor_1_pos, 
                                                    motor_2_pos};

    // Initialize timing params
    ros::Rate r(10);

    // for (int i = 0; i < motor_ids.size(); i++) {
    //     std::cout << "Motor " << motor_ids[i] << " Position:" <<std::endl;
    //     std::cout << "\t";
    //     for (int j = 0; j < motor_pos[i].size(); j++) {
    //         if (j == 0) {
    //             std::cout << "[ " << motor_pos[i][j];
    //         }
    //         else {
    //             std::cout << ", " << motor_pos[i][j];
    //         }
    //     }
    //     std::cout << "]" << std::endl;
    // }

    // Setup pubs and subs
    ros::Publisher leg_command_array_pub_ =
        nh_.advertise<quad_msgs::LegCommandArray>(leg_command_array_topic, 1);
    ros:: Publisher robot_heartbeat_pub_ =
        nh_.advertise<std_msgs::Header>(robot_heartbeat_topic, 1);
    
    //Subscribe to teleop twist joy
    ros::Subscriber sensor_msgs_sub = nh_.subscribe("/joy", 1, sensorMsgCallback);
    
    /// Message for leg command array
    quad_msgs::LegCommandArray leg_command_array_msg_;
    leg_command_array_msg_.leg_commands.resize(4);

    bool SENDING_COMMANDS = true;
    int i = 1;

    uint64_t currTime = getCurrTime();
    uint64_t prevTime = currTime;
    uint64_t waitTime = 1000;

    int indx = 0;
    int kp = 5;
    int kd = 1;

    ROS_INFO("Starting command loop\n");

    while(ros::ok) {
        // Collect new messages on subscriber topics and publish heartbeat
        ros::spinOnce();
        currTime = getCurrTime();
        if (currTime - prevTime > waitTime) {
            // Publish new command
            for (int i = 0; i < numFeet; ++i) {
                leg_command_array_msg_.leg_commands.at(i).motor_commands.resize(3);

                for (int j = 0; j < 3; ++j) {
                    int joint_idx = 3 * i + j;

                    float goalPos = 0;
                    if (j == currAxis) {
                        goalPos = joyPos;
                        ROS_INFO("Sending command %f to axis %i\n", goalPos, currAxis);
                    }

                    robot_driver_utils::loadMotorCommandMsg(
                                    goalPos, 0, 0, kp, kd,
                                    leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j));
                    
                    leg_command_array_msg_.leg_commands.at(i)
                        .motor_commands.at(j)
                        .pos_component = 0;
                    leg_command_array_msg_.leg_commands.at(i)
                        .motor_commands.at(j)
                        .vel_component = 0;
                    leg_command_array_msg_.leg_commands.at(i)
                        .motor_commands.at(j)
                        .fb_component = 0;
                    leg_command_array_msg_.leg_commands.at(i)
                        .motor_commands.at(j).effort = 0;
                    leg_command_array_msg_.leg_commands.at(i)
                        .motor_commands.at(j).fb_ratio = 0;
                }
            }
            leg_command_array_msg_.header.stamp = ros::Time::now();
            leg_command_array_pub_.publish(leg_command_array_msg_);
            indx = indx + 1;
            if (indx > motor_0_pos.size()) indx = 0;
        }
        r.sleep();
    }

    return 0;
}
