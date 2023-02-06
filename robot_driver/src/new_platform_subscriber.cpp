#include <sensor_msgs/Joy.h>

#include "robot_driver/robot_driver.h"

using namespace std::chrono;

int numFeet = 1;
int currAxis = 0;
float joyPos = 0;

uint64_t getCurrTime() {
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "new_platform_subscriber_node");
    ros::NodeHandle nh_;

    // //Create instance of new_platform_interface
    NewPlatformInterface newPlatformInterface;

    // newPlatformInterface::loadInterface();

    // Initialize timing params
    ros::Rate r(10);

    // // Setup pubs and subs
    // ros::Subscriber leg_command_array_sub =
    //     nh_.subscribe("topics/control/joint_command", 1, &newPlaftormInterface::send);

    while(ros::ok) {
        ros::spinOnce();



        r.sleep();
    }

    newPlatformInterface::unloadInterface();

    return 0;
}