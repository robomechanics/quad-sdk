/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Confidential, proprietary and/or trade secret materials.
 * No Distribution without prior written approval.
 *
 * Written by Tom Jacobs <tom.jacobs@ghostrobotics.io>
 *
 * This sends control messages to the Ghost Robotics robots via ethernet UDP MAVLink packets.
 * 
 * This makes the Vision 60 or Spirit 40 robot stand up, then look left, look right, look up, look down, lower its height, roll its body right, then sit down.
 *
 * NOTE: Set your computer IP address to 192.168.168.5, or change the UPST_ADDRESS to your IP, below.
 * 
 * Updated: July 2020
 *
 */

// Includes
#include <stdio.h>
#include <thread>
#include <chrono>

// MAVLink API
#include <mblink/mblink.hpp>

// Using
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using gr::MBLink;
typedef std::unordered_map<std::string, Eigen::VectorXf> RxData_t;
struct LimbCmd_t {
  // float pos[3], Kp[3], Kd[3];
  // probs add tau
    // float tau[3], pos[3], vel[3], Kp[3], Kd[3];
    float pos[3], Kp[3], Kd[3];
};
LimbCmd_t limbcmd[4];
float data[58];
int main(int argc, char *argv[])
{
    // constructer
    MBLink mblink;
    mblink.start(argc, argv);
    mblink.rxstart();
    mblink.setRetry("UPST_ADDRESS", 5);
    //

    // Stop Xavier/TX2 MAVLink packets.
    printf("\n*** First, make sure the Xavier/TX2 is not sending conflicting MAVLink commands, by running:\n\n ssh ghost@192.168.168.105\n sudo service ghost stop\n\n");

    // Stop app from sending MAVLink packets.
    printf("*** Then, close the remote controller app on the phone to make sure it is not running and sending conflicting MAVLink commands.\n\n");

    // Put robot into MAVLINK COMMANDS MODE.
    printf("*** Note: If you have a Taranis RC remote, toggle the front left long toggle switch from up/away (MANUAL MODE) to down/towards (MAVLINK COMMANDS MODE)\n\n");

    // Ask the robot mainboard to set the upstream address, to send to 192.168.168.5, the IP address of the computer this is running on.
    // Change 5 to the last digit of your IP address:
    

    // Stand - keep sending it until standing, so no 1 second comms timeout is hit, and send it once more at top of stand to enter lookaround mode

    // Look left
    float tau_des = 1;
    float pos_des = 0.5;
    float vel_des = 1;
    float Kp = 20;
    float Kd = 5;
    float commanded_positions[] = {0,0,0,0,0,0,0,0,0,0,0,0};
    // float commanded_positions[] = {0.943,1.943,-1.860,1.551,0.930,1.991,-1.771,2.102,0.704,0.643,-0.763,-0.577};
    for(int n = 0; n < 3; n++){
        for(int i = 0; i< 3;i++){
            // limbcmd[n].tau[i] = tau_des;
            limbcmd[n].pos[i] = commanded_positions[i+n];
            // limbcmd[n].vel[i] = vel_des;
            limbcmd[n].Kp[i] = Kp;
            limbcmd[n].Kd[i] = Kd;
        }
    }
    memcpy(data,limbcmd,4 * sizeof(LimbCmd_t));
    for(int ii = 0; ii<100; ii++){
    // float data[] = {tau_des,pos_des,vel_des,Kp,Kd};
    mblink.sendUser(Eigen::Map<const Eigen::Matrix<float, 58, 1> >(data));

    // mblink.sendUser(data);
    sleep_for(milliseconds(500));

    // RxData_t data = mblink.get();
    // // float current_data = data["user"][0];
    // float current_data = data["y"][20];
    // printf("Output of host: %f",limbcmd[0].pos[0]);
    // printf("\t Output of mainboard: %f",current_data);
    // printf("\n");

    }
    // End function
    mblink.rxstop();
    //

    return 0;
}
