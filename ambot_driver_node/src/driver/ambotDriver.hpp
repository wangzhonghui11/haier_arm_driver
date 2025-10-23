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
#include "bimax_msgs/msg/motor_command.hpp"
#include "bimax_msgs/msg/motor_state.hpp"
#include "bimax_msgs/msg/robot_command.hpp"
#include "bimax_msgs/msg/robot_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "bimax_msgs/srv/led_control.hpp"  
#include "bimax_msgs/srv/magnet_control.hpp"  
#include "bimax_msgs/srv/catcher_control.hpp"
#include "bimax_msgs/srv/mop_control.hpp"    
namespace ambot_driver_ns{
     #define LEN_MAX 4096
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
        bool JawCommandProcess(float pos);
        bool CommandFrameProcess(bimax_msgs::msg::RobotCommand& cmd);
        bool CommandServeLedProcess(uint8_t green,uint8_t yellow);
        bool CommandServeMagnetProcess(uint8_t green,uint8_t yellow);
        bool CommandServeCatcherProcess(uint8_t green,uint8_t yellow);
        bool CommandServeMopProcess(uint16_t mop_motor_pwm,uint8_t mop_state);
    private:
        void  floatToUint32(float input, uint8_t* des);
        PrivateProtocolCLASS *protocol;         //communication protocol instance
        const RobotDriver_TP robotParams;       //the const input robot params
        void printByteStream(const uint8_t* data, ssize_t count);
        int motorFd, sensorFd;                  //low driver motor file ID and sensor ID
        pthread_t motorTid, sensorTid;          //motor read feedback thread ID and sensor read thread ID
        size_t processBuffer(const uint8_t* data, size_t length) ;
        bool LifterMotorprocess(bimax_msgs::msg::RobotCommand& cmd);
        ssize_t txPacket(protocolOutputBuffer_TP &out);
        bool setMotorLocomotionCommand(CommFrame* frame) ;       
        void getAllMotorStateFromMCU(void);
        static void* newReadMotorThread(void* arg);
        void createReceiveThread(void);
        static void* newProccessMotorThread(void* arg);
        ssize_t printReceivedDataWithFrequency(int motorFd) ;
        void ProccessAllMotorStateFromMCU(void);
        bool YiyouMotorprocess(bimax_msgs::msg::RobotCommand& cmd);

    };
    
}


#endif
