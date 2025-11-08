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
        AmbotDriverCLASS(const std::shared_ptr<RosClass>& ros);
        ~AmbotDriverCLASS();
        /* open function */
        void  setLifterLeftPos(float new_pos);  
        void  setLifterRightPos(float pos);
        void  setJawPosition(float pos) {jawPos.store(pos, std::memory_order_release);} 
        bool  initial(void);
        bool  jawCommandProcess(float pos);
        bool  commandFrameProcess(bimax_msgs::msg::RobotCommand& cmd);
        bool  commandServeLedProcess(uint8_t green,uint8_t yellow);
        bool  commandServeMagnetProcess(uint8_t green,uint8_t yellow);
        bool  commandServeCatcherProcess(uint8_t green,uint8_t yellow);
        bool  commandServeMopProcess(uint16_t mop_motor_pwm,uint8_t mop_state);
        bool  commandSetTimeProcess(uint64_t time);
        float getJawPos() const { return jawPos.load(std::memory_order_acquire); }
        float getLifterLeftPos() const {return lifterLiftPos.load(std::memory_order_acquire);}         
        float getLifterRightPos() const {  return lifterRightPos.load(std::memory_order_acquire);}     

    private:
        int motorFd, sensorFd;    
        std::shared_ptr<bimax_driver_ns::RosClass> ros;
        std::atomic<float>lifterLiftPos{0.0f};
        std::atomic<float>lifterRightPos{0.0f};
        std::atomic<float> jawPos{0.0f};
        pthread_t motorTid, sensorTid;          //motor read feedback thread ID and sensor read thread ID
        PrivateProtocolCLASS *protocol;         //communication protocol instance
        
        void floatToUint32(float input, uint8_t* des);
        void printByteStream(const uint8_t* data, ssize_t count);
        void getAllMotorStateFromMCU(void);
        void createReceiveThread(void);
        void proccessAllMotorStateFromMCU(void);
        void checkToMotorStates(YiyouMecArm &mecarm);
        bool setMotorLocomotionCommand(CommFrame* frame) ;  
        bool lifterMotorProcess(bimax_msgs::msg::RobotCommand& cmd);
        bool yiyouMotorProcess(bimax_msgs::msg::RobotCommand& cmd);
        static void* newReadMotorThread(void* arg);
        static void* newProccessMotorThread(void* arg);
        size_t processBuffer(const uint8_t* data, size_t length) ;
        ssize_t txPacket(protocolOutputBuffer_TP &out);
        ssize_t printReceivedDataWithFrequency(int motorFd) ;     







    };
    
}


#endif
