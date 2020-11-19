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
#include <numeric>  

// MAVLink API
#include <mblink/mblink.hpp>

// Using
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using gr::MBLink;

typedef std::unordered_map<std::string, Eigen::VectorXf> RxData_t;

double median(std::vector<double> &v)
{
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin()+n, v.end());
    return v[n];
}

double std_dev(std::vector<double> &v)
{
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();

    std::vector<double> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(), [mean](double x) { return x - mean; });
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / v.size());
    return stdev;
}

int main(int argc, char *argv[])
{

    // Initialize mblink
    MBLink mblink;
    mblink.start(argc, argv);
    mblink.rxstart();

    // Initialize timing vars
    auto start = std::chrono::steady_clock::now();
    auto previous_comp = std::chrono::steady_clock::now();
    auto current_comp = std::chrono::steady_clock::now();
    double previous_robot = 0;
    double current_robot = 0;
    auto total_elapsed_comp = std::chrono::duration_cast<std::chrono::microseconds>(current_comp-start).count();
    std::vector<double> time_data_comp;
    std::vector<double> time_data_robot;

    // Stop Xavier/TX2 MAVLink packets.
    printf("\n*** First, make sure the Xavier/TX2 is not broadcasting conflicting MAVLink commands, by running:\n\n ssh ghost@192.168.168.105\n sudo service ghost stop\n\n");

    // Put robot into MAVLINK COMMANDS MODE.
    printf("*** Then, on the RC remote, toggle the front left long toggle switch from up/away (MANUAL MODE) to down/towards (MAVLINK COMMANDS MODE)\n\n");

    // Ask the robot mainboard to set the upstream address, to send to 192.168.168.5, the IP address of the computer this is running on.
    // Change 5 to the last digit of your IP address:
    mblink.setRetry("UPST_ADDRESS", 5);
    // mblink.setRetry("UPST_LOOP_DELAY", 3);
    mblink.setRetry("UPST_LOOP_DELAY", 20);

    while(total_elapsed_comp <= 10*1e6)
    {
        // Get sensor data and time it
        previous_comp = std::chrono::steady_clock::now();
        RxData_t data = mblink.get();
        
        // Log the current times
        current_comp = std::chrono::steady_clock::now();
        current_robot = data["y"][20];
        if (previous_robot==0)
            previous_robot = current_robot;

        // Update the elapsed time
        double current_elapsed_comp = (double)(std::chrono::duration_cast<std::chrono::microseconds>(current_comp-previous_comp).count())/1e6;
        double current_elapsed_robot = current_robot - previous_robot;
        time_data_comp.push_back(current_elapsed_comp);
        time_data_robot.push_back(current_elapsed_robot);

        // Print out the elapsed computer and mainboard time
        printf("Comp time elapsed: %6.2f ms \t", current_elapsed_comp*1e3);
        printf("Robot time elapsed: %8.2f ms \t", current_elapsed_robot*1e3);
        printf("User data check: %f\t", data["user"][1]);
        // printf("Robot time current: %10.6f\t", current_robot);
        // printf("Robot time previous: %10.6f\n", previous_robot);
        printf("\n");

        // Reset the counters
        total_elapsed_comp = std::chrono::duration_cast<std::chrono::microseconds>(current_comp-start).count();
        previous_robot = current_robot;
    }

    mblink.rxstop();

    std::sort(time_data_comp.begin(), time_data_comp.end());
    std::sort(time_data_robot.begin(), time_data_robot.end());
    double mean_time_comp = std::accumulate( time_data_comp.begin(), time_data_comp.end(), 0.0)/time_data_comp.size();   
    double mean_time_robot = std::accumulate( time_data_robot.begin(), time_data_robot.end(), 0.0)/time_data_robot.size();   
    printf("Avg comp time elapsed: %f, avg robot time elapsed: %f \n", mean_time_comp, mean_time_robot);
    printf("Std comp time elapsed: %f, std robot time elapsed: %f \n", std_dev(time_data_comp), std_dev(time_data_robot));
    printf("Med comp time elapsed: %f, med robot time elapsed: %f \n", median(time_data_comp), median(time_data_robot));
    printf("Max comp time elapsed: %f, max robot time elapsed: %f \n", time_data_comp.back(),time_data_robot.back());

    return 0;
}
