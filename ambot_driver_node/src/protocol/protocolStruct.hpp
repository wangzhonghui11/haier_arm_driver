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

    constexpr uint8_t FRAME_HEAD_H = 0xF0;
    constexpr uint8_t FRAME_HEAD_L = 0xF0;
    /*this enum indicate all function code  according to our private protocol*/

        // 存储编号
        enum class StoreNum : uint8_t {
            TIME_SET    = 0,
            MOP_SET     = 1,
            JAW_SET     = 2,
            CATCHER_SET = 3,
            MAGNET_SET  = 4,
            MECARM_SET  = 5,
            LIFTS_SET   = 6,
            LED_SET     = 7
        };

        // 数据长度定义
        constexpr uint8_t DATA_LENGTH_TIME_SET    = 8;
        constexpr uint8_t DATA_LENGTH_MOP_SET     = 3;
        constexpr uint8_t DATA_LENGTH_JAW_SET     = 4;
        constexpr uint8_t DATA_LENGTH_CATCHER_SET = 4;
        constexpr uint8_t DATA_LENGTH_MAGNET_SET  = 2;
        constexpr uint8_t DATA_LENGTH_MECARM_SET  = 1;
        constexpr uint8_t DATA_LENGTH_LIFTS_SET   = 1;
        constexpr uint8_t DATA_LENGTH_LED_SET     = 2;
        constexpr uint8_t DATA_LENGTH_HEARTBEAT   = 0;

        // MCU发送给导航的上传ID
        enum class UploadID : uint8_t {
            MEC_ARM_UPLOAD   = 0x11,
            LIFTS_UPLOAD     = 0x12,
            JAW_MOTOR_UPLOAD = 0x13
        };

        // 上传数据长度
        constexpr uint8_t DATA_LENGTH_MEC_ARM   = 4;
        constexpr uint8_t DATA_LENGTH_LIFTS     = 4;
        constexpr uint8_t DATA_LENGTH_JAW_MOTOR = 6;

        // 定时器结构体
        struct TimerHostComm {
            uint64_t OSTimer_HostComm;
            uint64_t MopLastTime;
            uint64_t JawLastTime;
            uint64_t CatcherLastTime;
            uint64_t MagnetLastTime;
            uint64_t MecarmLastTime;
            uint64_t lifts_LastTime;
            uint64_t HostComm_Seting_Time;
            uint64_t HostComm_Time_NOW;
            uint64_t HostComm_Time_PRE;
            uint64_t HeartbeatLastTime;
        };

        // 心跳检测结构体
        struct HeartBeat {
            uint8_t cnt;
            uint8_t wait;
            uint8_t TimeOut;
        };

        // 时间设置数据结构
        struct TimeSet {
            union {
                struct {
                    uint64_t time;
                } real;
                uint8_t data[8];
            };
        };

        // 拖把设置数据结构
        struct MopSet {
            union {
                struct {
                    uint16_t mop_motor_pwm;  // 0-1000
                    uint8_t  mop_state;      // 1或0
                } real;
                uint8_t data[3];
            };
        };

        // 夹爪设置数据结构
        struct JawSet {
            union {
                struct {
                    float jaw_motor_angle;  // 0-90度
                } real;
                uint8_t data[4];
            };
        };

        // 捕捉器设置数据结构
        struct CatcherSet {
            union {
                struct {
                    uint16_t catcher_motor_pwm;  // 0-1000
                    uint8_t  catcher_gear;       // 2,1或0
                    uint8_t  catcher_state;      // 1或0
                } real;
                uint8_t data[4];
            };
        };

        // 磁铁设置数据结构
        struct MagnetSet {
            union {
                struct {
                    uint8_t left_magnet_state;   // 1或0
                    uint8_t right_magnet_state;  // 1或0
                } real;
                uint8_t data[2];
            };
        };

        // 机械臂设置数据结构
        struct MecArmSet {
            union {
                struct {
                    uint8_t null;  // 未使用
                } real;
                uint8_t data;
            };
        };

        // 升降机构设置数据结构
        struct LiftsSet {
            union {
                struct {
                    uint8_t null;  // 未使用
                } real;
                uint8_t data;
            };
        };

        // LED设置数据结构
        struct LedSet {
            union {
                struct {
                    uint8_t led_green_state;   // 1,0或2(闪烁)
                    uint8_t led_yellow_state;  // 1,0或2(闪烁)
                } real;
                uint8_t data[2];
            };
        };

        // 心跳数据结构
        struct Heartbeat {
            union {
                struct {
                    uint8_t reserve;
                } real;
                uint8_t data;
            };
        };

        // 机械臂上传数据结构
        struct UploadMecArm {
            union {
                struct {
                    uint16_t time;
                    uint16_t reserve;  // 预留
                } real;
                uint8_t data[4];
            };
        };

        // 升降机构上传数据结构
        struct UploadLifts {
            union {
                struct {
                    uint16_t time;
                    uint16_t reserve;  // 预留
                } real;
                uint8_t data[4];
            };
        };

        // 夹爪上传数据结构
        struct UploadJaw {
            union {
                struct {
                    uint16_t time;
                    float jaw_motor_angle;  // 0-180度
                } real;
                uint8_t data[6];
            };
        };

        // 通用串口消息帧结构
        struct CommFrame {
            uint8_t head_H;      // 帧头0xF0F0
            uint8_t head_L;
            uint8_t frame_ID_H;  // 帧ID
            uint8_t frame_ID_L;
            uint8_t length;      // 数据长度 = cmd_ID(1byte)+数据长度
            uint8_t cmd_ID;      // 命令ID
            uint8_t* databuf;    // 数据缓冲区
            uint8_t CRC_H;       // CRC16高字节
            uint8_t CRC_L;       // CRC16低字节
        };

        // 初始化全局变量
        uint8_t PerComm_Step = 0;
        uint8_t HostComm_Step = 0;
        uint16_t crc_rslt = 0;

        // 缓冲区初始化
        uint8_t HostComm_Rx_buff[32] = {0};  // 接收数据数组
        uint8_t HostComm_Tx_buff[32] = {0};  // 发送数据数组
        static uint8_t databuf_temp[32] = {0};

        // 定时器和心跳结构体初始化
        TimerHostComm Timer_HostComm = {};
        // HeartBeat HeartBeat = {};

        /*****************************************************/ 
        // MCU发送给导航的数据结构初始化
        UploadMecArm commmecarm = {};
        UploadLifts commlifts = {};
        UploadJaw commjaw = {};

        // MCU发送给导航的帧结构初始化
        CommFrame uploadframeMecarm = {
            .head_H = FRAME_HEAD_H,
            .head_L = FRAME_HEAD_L,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LENGTH_MEC_ARM,
            .cmd_ID = static_cast<uint8_t>(UploadID::MEC_ARM_UPLOAD),
            .databuf = commmecarm.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame uploadframeLifts = {
            .head_H = FRAME_HEAD_H,
            .head_L = FRAME_HEAD_L,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LENGTH_LIFTS,
            .cmd_ID = static_cast<uint8_t>(UploadID::LIFTS_UPLOAD),
            .databuf = commlifts.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame uploadframeJaw = {
            .head_H = FRAME_HEAD_H,
            .head_L = FRAME_HEAD_L,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LENGTH_JAW_MOTOR,
            .cmd_ID = static_cast<uint8_t>(UploadID::JAW_MOTOR_UPLOAD),
            .databuf = commjaw.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        /*****************************************************/ 
        // 导航发送给MCU的数据结构初始化
        TimeSet dataTimeSet = {};
        MopSet dataMopSet = {};
        JawSet dataJawSet = {};
        CatcherSet dataCatcherSet = {};
        MagnetSet dataMagnetSet = {};
        MecArmSet dataMecArmSet = {};
        LiftsSet dataLiftsSet = {};
        LedSet dataLedSet = {};
        Heartbeat dataHeartbeat = {};

        // 导航发送给MCU的帧结构初始化
        CommFrame cmdframTimeSet = {
            .head_H = 0,
            .head_L = 0,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LENGTH_TIME_SET,
            .cmd_ID = static_cast<uint8_t>(StoreNum::TIME_SET),
            .databuf = dataTimeSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframMopSet = {
            .head_H = 0,
            .head_L = 0,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LENGTH_MOP_SET,
            .cmd_ID = static_cast<uint8_t>(StoreNum::MOP_SET),
            .databuf = dataMopSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframJawSet = {
            .head_H = 0,
            .head_L = 0,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LENGTH_JAW_SET,
            .cmd_ID = static_cast<uint8_t>(StoreNum::JAW_SET),
            .databuf = dataJawSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframCatcherSet = {
            .head_H = 0,
            .head_L = 0,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LENGTH_CATCHER_SET,
            .cmd_ID = static_cast<uint8_t>(StoreNum::CATCHER_SET),
            .databuf = dataCatcherSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframMagnetet = {
            .head_H = 0,
            .head_L = 0,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LENGTH_MAGNET_SET,
            .cmd_ID = static_cast<uint8_t>(StoreNum::MAGNET_SET),
            .databuf = dataMagnetSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframMecArmSet = {
            .head_H = 0,
            .head_L = 0,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LENGTH_MECARM_SET,
            .cmd_ID = static_cast<uint8_t>(StoreNum::MECARM_SET),
            .databuf = &dataMecArmSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframLiftsSet = {
            .head_H = 0,
            .head_L = 0,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LENGTH_LIFTS_SET,
            .cmd_ID = static_cast<uint8_t>(StoreNum::LIFTS_SET),
            .databuf = &dataLiftsSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframLedSet = {
            .head_H = 0,
            .head_L = 0,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LENGTH_LED_SET,
            .cmd_ID = static_cast<uint8_t>(StoreNum::LED_SET),
            .databuf = dataLedSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        // CommFrame cmdframHeartbeat = {
        //     .head_H = 0,
        //     .head_L = 0,
        //     .frame_ID_H = 0,
        //     .frame_ID_L = 0,
        //     .length = 1 + DATA_LENGTH_HEARTBEAT,
        //     .cmd_ID = static_cast<uint8_t>(StoreNum::HEARTBEAT),
        //     .databuf = &dataHeartbeat.data,
        //     .CRC_H = 0,
        //     .CRC_L = 0
        // };

        // 命令帧组初始化
        CommFrame* cmdframGroup[8] = {
            &cmdframTimeSet,
            &cmdframMopSet,
            &cmdframJawSet,
            &cmdframCatcherSet,
            &cmdframMagnetet,
            &cmdframMecArmSet,
            &cmdframLiftsSet,
            &cmdframLedSet
        };    
    
}

#endif 