#include "robot_driver/hardware_interfaces/CANInterface.hpp"

namespace CAN_interface
{
    CANInterface::CANInterface(const char* socketName)
    {
        struct sockaddr_can addr; // structure for CAN sockets : address family number AF_CAN
        struct ifreq ifr; // from if.h Interface Request structure used for all socket ioctl's. All interface ioctl's must have parameter definitions which begin with ifr name. The remainder may be interface specific.

        int loopback = 0; /* 0 = disabled, 1 = enabled (default) */

        // socket(int domain, int type, int protocol): returns file descriptor int or -1 if fail
        if ((socket_descrp_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
        {
            ROS_ERROR("CANInterface: Error While Opening CAN Socket");
        }
        else {
            // If socket was created successfully, apply the can filter for only receiving from motor and not from master.
            // setsockopt(socket_descrp_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

            setsockopt(socket_descrp_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

            // Retrieve the interface index for the interface name (can0, can1, vcan0) to be used to the ifreq struct
            strcpy(ifr.ifr_name, socketName);

            // Send an I/O control call and pass an ifreq structure containing the interface name
            // ioctl() system call manipulates the underlying device parameters of special files. 
            // SIOCGIFINDEX Retrieve the interface index of the interface into ifr_ifindex inside ifr struct.
            ioctl(socket_descrp_, SIOCGIFINDEX, &ifr);

            // with the interface index, now bind the socket to the CAN Interface
            struct sockaddr_can addr;

            // set address to all zeros. Done in example/man pages. But why?
            memset(&addr, 0, sizeof(addr));

            // Setup the interface parameters in the socketcan address struct
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;

            if (bind(socket_descrp_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
            {
               ROS_ERROR("CANInterface: Error while binding to the CAN Socket.");
            }
        }
    }

    bool CANInterface::sendCANFrame(int can_id, const unsigned char* CANMsg)
    {
        struct can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = 8;
        memcpy(frame.data, CANMsg, 8);

        if (write(socket_descrp_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            ROS_ERROR("CANInterface: Error writing to CAN Interface.");
            return false;
        }
        else
        {
            return true;
        }
    }

    // bool CANInterface::receiveCANFrame(unsigned char* CANMsg)
    // {
    //     // Listen to all CAN messages. Filter by Motor ID later in the motor driver class.
    //     struct can_frame frame;

    //     if (read(socket_descrp_, &frame, sizeof(struct can_frame)) < 0)
    //     {
    //         perror("CANInterface: Error Reading Data.");
    //         return false;
    //     }
    //     else
    //     {
    //         memcpy(CANMsg, frame.data, frame.can_dlc);
    //         return true;
    //     }
    // }

    int CANInterface::receiveCANFrame(unsigned char* CANMsg, std::chrono::milliseconds timeout_time)
    {
        // Listen to all CAN messages. Filter by Motor ID later in the motor driver class.
        struct can_frame frame;

        std::mutex m;
        std::condition_variable cv;
        int retValue;

        std::thread t([&cv, &retValue, this, &frame]() 
        {
            ROS_INFO("Reading Data");
            retValue = read(socket_descrp_, &frame, sizeof(struct can_frame));
            cv.notify_one();
        });

        t.detach();

        {
            std::unique_lock<std::mutex> l(m);
            if(cv.wait_for(l, timeout_time) == std::cv_status::timeout) {
                ROS_WARN("Recieve CAN frame timed out.\n");
                return Recieve_CAN_Frame::TIMEOUT;
            }
        }
        
        if (retValue < 0)  // getting stuck here
        {
            ROS_ERROR("CANInterface: Error Reading Data.");
            return Recieve_CAN_Frame::FAILED;
        }
        else
        {
            memcpy(CANMsg, frame.data, frame.can_dlc);
            return Recieve_CAN_Frame::SUCCESS;
        }
    }
    CANInterface::~CANInterface() {

        if (close(socket_descrp_) < 0) {
            ROS_ERROR("CANInterface: Error Closing CAN Socket.");
        }

    }
}