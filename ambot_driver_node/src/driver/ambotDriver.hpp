/*  A class include yangtze river inspection robot hardware driver
    *
    * Author： chen chen
    * Date: 2024-8-22
    * Email: 1240563221@qq.com
    *
*/

#ifndef __YANGTZE_ROBOT_HPP__
#define __YANGTZE_ROBOT_HPP__

#include <iostream>
#include <vector>
#include "unistd.h"
#include <string.h>
#include <linux/serial.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h> 
#include <pthread.h>
#include <type_traits>
#include <sys/time.h>
#include <iomanip>
#include "protocolStruct.hpp"
#include "privateProtocol.hpp"
#include <chrono>
namespace ambot_driver_ns{

    typedef struct 
    {
        std::string robotType;
        std::string motorDevName;
        std::string sensorDevName;
        unsigned int motorDevBaud;
        unsigned int sensorDevBaud;
        uint8_t jointMotorNum;
        uint8_t wheelMotorNum;
        uint8_t motorNum;
        std::vector<uint8_t> motorControlMode;
        std::vector<float> motorMaxAngle;
        std::vector<float> motorMinAngle;
        std::vector<float> motorVelocity;
        std::vector<float> motorOffset;
        std::vector<float> motorAxisDirection;
        uint8_t forceDataNum;
        uint8_t sensorNum;
        uint16_t rosRate;
        float   carMaxVel;
        float   wheelRadius;
    }RobotDriver_TP;

    class AmbotDriverCLASS
    {
    public:
        bool threadStop;
        std::vector<std::string> robotType;

        AmbotDriverCLASS(RobotDriver_TP &input);
        ~AmbotDriverCLASS();
        /* open function */
        bool initial(void);

    private:
        PrivateProtocolCLASS *protocol;         //communication protocol instance
        const RobotDriver_TP robotParams;       //the const input robot params
        void printByteStream(const uint8_t* data, ssize_t count);
        int motorFd, sensorFd;                  //low driver motor file ID and sensor ID
        pthread_t motorTid, sensorTid;          //motor read feedback thread ID and sensor read thread ID
        size_t processBuffer(const uint8_t* data, size_t length) ;

        ssize_t txPacket(protocolOutputBuffer_TP &out);

        void getAllMotorStateFromMCU(void);
        static void* newReadMotorThread(void* arg);
        void createReceiveThread(void);
        static void* newProccessMotorThread(void* arg);
        ssize_t printReceivedDataWithFrequency(int motorFd) ;
        void ProccessAllMotorStateFromMCU(void);


    };
    
}


#endif
