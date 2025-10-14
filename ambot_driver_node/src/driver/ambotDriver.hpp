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
// #include "privateProtocol.hpp"

// #include <ambot_msg/JointState.h>
// #include <ambot_msg/AmbotCommand.h>
// #include <ambot_msg/AmbotState.h>

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
        // ambot_msg::AmbotState rosData;          //ros message data, wait to publish 
        /* construct and deconstruct function */
        AmbotDriverCLASS(RobotDriver_TP &input);
        ~AmbotDriverCLASS();

        /* open function */
        bool initial(void);
        // bool disableAllMotor(void);
        // bool setMotorLocomotionCommand(const ambot_msg::AmbotCommand &command, const std::vector<float> &wheel);
        // void getWheelVelocity(std::vector<float> &out);
    private:
        // PrivateProtocolCLASS *protocol;         //communication protocol instance
        const RobotDriver_TP robotParams;       //the const input robot params

         int motorFd, sensorFd;                  //low driver motor file ID and sensor ID
         pthread_t motorTid, sensorTid;          //motor read feedback thread ID and sensor read thread ID
        
        // std::vector<int> motorIDExist;          //current exist motor ID     
        // int motorNumExist;                      //current exist motor num

        // /*control command frame build and receive variables*/
        // std::vector<MotorFB_TP> motorFB;        //current exist motor feedback data
        // std::vector<float> wheelVel;            

        // SensorFB_TP sensorFB;                   //sensor data, include imu and etc....
        // protocolInputBuffer_TP motorCommandSum;     //all motor command data
        // protocolInputBuffer_TP motorCommandExist;   //current exist motor command data 

        // ssize_t txPacket(protocolOutputBuffer_TP &out);
        // bool waitCommandResponse(const uint8_t commandNumber);
        // bool waitRequestIDResponse(const uint8_t commandNumber, std::vector<int> &id);
        // template<typename T> bool setAllMotorOneFeature(const uint8_t inputCommandCode, const std::vector<int> &IDBuffer, const std::vector<T> &inputData);
        // bool requestAllMotorID(std::vector<int> &out);
        // //build a new thread to receive and do data analysis 
        // void motorRawState2RosPublish(std::vector<MotorFB_TP> &input, ambot_msg::AmbotState &output);
        // void sensorRaw2RosPublish(const SensorFB_TP &in, ambot_msg::AmbotState &out);
        void getAllMotorStateFromMCU(void);
        // void getSensorDataFromMCU(void);
         static void* newReadMotorThread(void* arg);
        // static void* newReadSensorThread(void* arg);
         void createReceiveThread(void);

        // bool checkAllMotorException(void);
        // void updateJointCmd(const ambot_msg::AmbotCommand& in);
        // void updateWheelCmd(const std::vector<float> &in);
        // void updateExistMotorCmd(void);
        // void updateWheelFb(void);
    };
    
}


#endif
