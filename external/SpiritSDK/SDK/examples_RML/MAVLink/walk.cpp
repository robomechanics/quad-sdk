/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Confidential, proprietary and/or trade secret materials.
 * No Distribution without prior written approval.
 *
 * Written by Tom Jacobs <tom.jacobs@ghostrobotics.io>
 *
 * This sends messages to the Ghost Robotics robots via ethernet UDP MAVLink packets.
 * 
 * This makes the Vision 60 or Spirit 40 robot stand up, walk around, then sit down.
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

int main(int argc, char *argv[])
{
    MBLink mblink;
    mblink.start(argc, argv);
    mblink.rxstart();

    // Stop Xavier/TX2 MAVLink packets.
    printf("\n*** First, make sure the Xavier/TX2 is not sending conflicting MAVLink commands, by running:\n\n ssh ghost@192.168.168.105\n sudo service ghost stop\n\n");

    // Stop app from sending MAVLink packets.
    printf("*** Then, close the remote controller app on the phone to make sure it is not running and sending conflicting MAVLink commands.\n\n");

    // Put robot into MAVLINK COMMANDS MODE.
    printf("*** Note: If you have a Taranis RC remote, toggle the front left long toggle switch from up/away (MANUAL MODE) to down/towards (MAVLINK COMMANDS MODE)\n\n");

    // Ask the robot mainboard to set the upstream address, to send to 192.168.168.5, the IP address of the computer this is running on.
    // Change 5 to the last digit of your IP address:
    mblink.setRetry("UPST_ADDRESS", 5);

    // Stand - keep sending it until standing, so no 1 second comms timeout is hit, and send it once more at top of stand to enter lookaround mode
    for(int i = 0; i < 15; i++)
    {
        printf("Sending: STAND to robot.\n"); // STAND, see: http://ghostrobotics.gitlab.io/docs/MAVLinkReference.html
        mblink.setBehavior(MBLink::STAND, 0);
        sleep_for(milliseconds(300));
    }

    // Enter walk mode
    printf("Sending: WALK to robot.\n");
    mblink.setBehavior(MBLink::WALK, 1); // WALK, see: http://ghostrobotics.gitlab.io/docs/MAVLinkReference.html
    sleep_for(milliseconds(500));

    // Walk forward
    printf("Forwards.\n");
    Eigen::Vector3f se2twist;
    se2twist << 0.2, 0.0, 0.0;
    for(int i = 0; i < 30; i++)
    {
        mblink.sendControl(se2twist);
        sleep_for(milliseconds(100));
    }

    // Walk backwards
    printf("Backwards.\n");
    se2twist << -0.2, 0.0, 0.0;
    for(int i = 0; i < 15; i++)
    {
        mblink.sendControl(se2twist);
        sleep_for(milliseconds(100));
    }

    // Turn left
    printf("Left.\n");
    se2twist << 0.0, 0.0, 0.4;
    for(int i = 0; i < 15; i++)
    {
        mblink.sendControl(se2twist);
        sleep_for(milliseconds(100));
    }

    // Turn right
    printf("Right.\n");
    se2twist << 0.0, 0.0, -0.4;
    for(int i = 0; i < 15; i++)
    {
        mblink.sendControl(se2twist);
        sleep_for(milliseconds(100));
    }

    // Sit
    printf("Sending: SIT to robot.\n");
    for(int i = 0; i < 4; i++)
    {
        mblink.setBehavior(MBLink::SIT, 0); // SIT, see: http://ghostrobotics.gitlab.io/docs/MAVLinkReference.html
        sleep_for(milliseconds(500));
    }
    
    mblink.rxstop();

    return 0;
}
