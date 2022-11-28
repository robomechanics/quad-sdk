#include "robot_driver/hardware_interfaces/CANInterface.hpp"


namespace CAN_interface
{
    CANInterface::CANInterface(const char* socketName)
    {
        // const char* socketIfName = &socketName;  
        // int s;  // File descriptor for the socket as everything in Linux/Unix is a file. 
        struct sockaddr_can addr; // structure for CAN sockets : address family number AF_CAN
        struct ifreq ifr; // from if.h Interface Request structure used for all socket ioctl's. All interface ioctl's must have parameter definitions which begin with ifr name. The remainder may be interface specific.

        int loopback = 0; /* 0 = disabled, 1 = enabled (default) */

        // socket(int domain, int type, int protocol): returns file descriptor int or -1 if fail
        if ((socket_descrp_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
        {
            perror("CANInterface: Error While Opening CAN Socket");
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
               perror("CANInterface: Error while binding to the CAN Socket.");
            }
            else
            {
                std::cout << "The Socket Descriptor is: " << socket_descrp_ << std::endl;
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
            perror("CANInterface: Error writing to CAN Interface.");
            return false;
        }
        else
        {
            return true;
        }
    }

    bool CANInterface::receiveCANFrame(unsigned char* CANMsg)
    {
        // Listen to all CAN messages. Filter by Motor ID later in the motor driver class.
        struct can_frame frame;

        if (read(socket_descrp_, &frame, sizeof(struct can_frame)) < 0)
        {
            perror("CANInterface: Error Reading Data.");
            return false;
        }
        else
        {
            memcpy(CANMsg, frame.data, frame.can_dlc);
            return true;
        }
    }

    CANInterface::~CANInterface() {

        if (close(socket_descrp_) < 0) {
            perror("CANInterface: Error Closing CAN Socket.");
        }

    }
}