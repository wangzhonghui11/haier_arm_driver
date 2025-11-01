/*  A class include yangtze river inspection robot hardware driver
    *
    * Author： chen chen
    * Date: 2024-8-22
    * Email: 1240563221@qq.com
    *
*/

#ifndef __YANGTZE_ROBOT_HPP__
#define __YANGTZE_ROBOT_HPP__
#include <thread>
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
#include "ambotRosClass.hpp"
    
namespace bimax_driver_ns{
     #define LEN_MAX 4096
    class AmbotDriverCLASS
    {
    public:

        bool threadStop;
        std::vector<std::string> robotType;
        YiyouMecArm mecarm = {
            .pos_rad_motor1 = 0.0f, .vel_rad_s_motor1 = 0.0f, .torque_nm_motor1 = 0.0f,
            .status_motor1 = 0xFF, .error_motor1 = 0,
            
            .pos_rad_motor2 = 0.0f, .vel_rad_s_motor2 = 0.0f, .torque_nm_motor2 = 0.0f,
            .status_motor2 = 0xFF, .error_motor2 = 0,
            
            .pos_rad_motor3 = 0.0f, .vel_rad_s_motor3 = 0.0f, .torque_nm_motor3 = 0.0f,
            .status_motor3 = 0xFF, .error_motor3 = 0,
            
            .pos_rad_motor4 = 0.0f, .vel_rad_s_motor4 = 0.0f, .torque_nm_motor4 = 0.0f,
            .status_motor4 = 0xFF, .error_motor4 = 0,
            
            .pos_rad_motor5 = 0.0f, .vel_rad_s_motor5 = 0.0f, .torque_nm_motor5 = 0.0f,
            .status_motor5 = 0xFF, .error_motor5 = 0
        };
        float lifter_l_pos;
        float  lifter_r_pos;
        float jaw_pos;
        AmbotDriverCLASS(const std::shared_ptr<RosClass>& ros);
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
        std::shared_ptr<bimax_driver_ns::RosClass> ros;
        void  floatToUint32(float input, uint8_t* des);
        PrivateProtocolCLASS *protocol;         //communication protocol instance
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
        void checkToMotorStates(YiyouMecArm &mecarm);

    };
    
}


#endif
