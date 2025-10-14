/*  A file include all private protocol struct and enum and macro
    *
    * Author： chen chen
    * Date: 2024-8-22
    * Email: 1240563221@qq.com
    *
*/

#ifndef __PROTOCOL_STRUCT_HPP__
#define __PROTOCOL_STRUCT_HPP__

#include <iostream>

namespace ambot_driver_ns
{
    /*these macro for global use*/
    #define WAIT_RESPONSE_US_DELAY  20              //us
    #define U_SECOND_COUNT          1000000         //us
    #define DELAY_TIMEOUT           3000000         //us
    #define CRC_DATA_SIZE           2           
    #define WRITE_BUFFER_SIZE       400         
    #define READ_BUFFER_SIZE        400         
    #define FRAME_HEAD_DATA         0xAA    
    #define MY_PI                   3.1415926f    

    #define COUT_RESET   "\033[0m"
    #define COUT_BLACK   "\033[30m"      /* Black */
    #define COUT_RED     "\033[31m"      /* Red */
    #define COUT_GREEN   "\033[32m"      /* Green */
    #define COUT_YELLOW  "\033[33m"      /* Yellow */
    #define COUT_BLUE    "\033[34m"      /* Blue */
    #define COUT_MAGENTA "\033[35m"      /* Magenta */
    #define COUT_CYAN    "\033[36m"      /* Cyan */
    #define COUT_WHITE   "\033[37m"      /* White */
    #define COUT_BOLD_BLACK   "\033[1m\033[30m"      /* Bold Black */
    #define COUT_BOLD_RED     "\033[1m\033[31m"      /* Bold Red */
    #define COUT_BOLD_GREEN   "\033[1m\033[32m"      /* Bold Green */
    #define COUT_BOLD_YELLOW  "\033[1m\033[33m"      /* Bold Yellow */
    #define COUT_BOLD_BLUE    "\033[1m\033[34m"      /* Bold Blue */
    #define COUT_BOLD_MAGENTA "\033[1m\033[35m"      /* Bold Magenta */
    #define COUT_BOLD_CYAN    "\033[1m\033[36m"      /* Bold Cyan */
    #define COUT_BOLD_WHITE   "\033[1m\033[37m"      /* Bold White */

    #define ambot_N1  0
    #define ambot_W1  1
    #define ambot_P1  2

    /*this enum indicate all function code  according to our private protocol*/
    typedef struct 
    {
        enum
        {
            CONTROL = 0x01,
            CONTROL_RESPOND,
            REQUEST,
            REQUEST_RESPOND,
            ALL_FUNCTION_CODE_NUMBER,
        };
    }FunctionCode_TP;

    /*this enum use for create tx data according to our private protocol*/
    typedef struct
    {
        enum
        {
            FRAME_HEAD = 0,
            FUNCTION_CODE,
            COMMAND_CODE,
            MOTOR_NUM,
            DATA_LENGTH,
            MOTOR_ID = 4,
            MOTOR_DATA = 5,
            MOTOR_DATA_SIZE = 5,
            LOCOMOTION_CONTROL_DATA_SIZE = 11,
        };
    }ControlIndex_TP;

    /*this enum use for analyses data according to our private protocol*/
    typedef struct
    {
        enum
        {
            FUNCTION_CODE = 0,
            COMMAND_CODE,
            CRC_AUTHENTICATION,
            MOTOR_NUM = 2,
            DATA_LENGTH,
            MOTOR_ID = 3,
            MOTOR_DATA_START,
            MOTOR_DATA_SIZE = 5,
        };
    }ResponseIndex_TP;

    /*this enum indicate all command code under control code  according to our private protocol*/
    typedef struct
    {
        enum
        {
            SET_ZERO = 0x01,
            ENABLE_MOTOR,
            DISABLE_MOTOR,
            SELECT_MODE,
            SET_MAX_ANGLE,
            SET_MIN_ANGLE,
            SET_RUNNING_VELOCITY,
            SET_POSITION,
            SET_CURRENT,
            SET_LOCOMOTION_CONTROL,
            ALL_COMMAND_NUMBER,
        };
    }ControlCommandCode_TP;

    typedef struct
    {
        enum
        {
            FUNCTION_CODE = 0,
            COMMAND_CODE,
            OFFLINE_MOTOR_HIGH,   //every bit indicate a motor id,from 1 to 12
            OFFLINE_MOTOR_LOW,   
            SERVE_1_HIGH,
            SERVE_1_LOW,
            SERVE_2_HIGH,
            SERVE_2_LOW,
            SERVE_3_HIGH,
            SERVE_3_LOW,
            ALL_DATA_NUM,
        };
    }MotorOffLineExceptionIndex_TP;

    typedef struct
    {
        enum
        {
            FUNCTION_CODE = 0,
            COMMAND_CODE,
            IMU_LINE_ACC_X,
            IMU_LINE_ACC_Y = 6,
            IMU_LINE_ACC_Z = 10,
            IMU_ANGULAR_X = 14,
            IMU_ANGULAR_Y = 18,
            IMU_ANGULAR_Z = 22,
            IMU_QUATERNION_W = 26,
            IMU_QUATERNION_X = 30,
            IMU_QUATERNION_Y = 34,
            IMU_QUATERNION_Z = 38,
            TOUCH_FL = 42,
            TOUCH_FR = 43,
            TOUCH_HL = 44,
            TOUCH_HR = 45,
            SHIFT_FL = 46,
            SHIFT_FR = 47,
            SHIFT_HL = 48,
            SHIFT_HR = 49,
            CRC_AUTHENTICATION = 50,
            ALL_DATA_NUM = 52,
        };
    }SensorDataIndex_TP;

    /*this enum indicate all command code under request code  according to our private protocol*/
    typedef struct
    {
        enum
        {
            GET_ID = 0x01,
            GET_STATUS,
            GET_POSITION,
            GET_CURRENT,
            GET_VELOCITY,
            GET_VOLTAGE,
            GET_SENSOR_IMU = 0x0B,
            MOTOR_EXCEPTION = 0x0C,
            ALL_REQUEST_NUMBER,
        };
    }RequestCommandCode_TP;

    /*this enum indicate all function code  according to our private protocol*/
    typedef struct 
    {
        enum
        {
            LOCOMOTION_MODE = 0x01,
            POSITION_MODE,
            VELOCITY_MODE,
            CURRENT_MODE,
            ALL_MODE_NUMBER,
        };
    }MotorRunMode_TP;

    typedef struct 
    {
        float maxPosition;
        float minPosition;
        float maxVelocity;
        float minVelocity;
        float maxCurrent;
        float minCurrent;
        float maxKp;
        float minKp;
        float maxKd;
        float minKd;
    }MotorLimitation_TP;

    /* protocol buffer typedef struct */
    typedef struct protocolOutputBuffer_TP
    {
        uint8_t buffer[WRITE_BUFFER_SIZE];
        uint16_t length;
    }protocolOutputBuffer_TP;
    
    /* protocol input data typedef struct */
    typedef struct MotorCommand_TP
    {
        uint8_t id;
        float q;
        float dq;
        float tor;
        float kp;
        float kd;
    }MotorCommand_TP;
    
    typedef struct protocolInputBuffer_TP
    {
        uint8_t number;
        std::vector<MotorCommand_TP> command;
    }protocolInputBuffer_TP;

    typedef struct MotorFB_TP
    {
        uint8_t id;
        float pos;
        float vel;
        float tor;
        uint16_t state;   
    }MotorFB_TP;

    typedef struct BaseIMU_TP
    {
        float x,y,z;
    }BaseIMU_TP;
    
    typedef struct QuaternionIMU_TP
    {
        float w,x,y,z;
    }QuaternionIMU_TP;

    
    typedef struct SensorFB_TP
    {
        BaseIMU_TP acceleration;
        BaseIMU_TP gyroscope;
        QuaternionIMU_TP quaternion;
    }SensorFB_TP;

    
    
}

#endif 