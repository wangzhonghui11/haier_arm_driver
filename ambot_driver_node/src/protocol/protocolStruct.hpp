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
 //#define DEBUG_MODE 1
namespace ambot_driver_ns
{
    /*these macro for global use*/

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
    #define WRITE_BUFFER_SIZE 4096
    #define ID_MEC_ARM_STORE  0
    #define ID_LIFTS_STORE  1
    #define ID_JAW_MOTOR_STORE 2
    #define FRAME_HEAD_H  0xF0
    #define FRAME_HEAD_L  0xF0

    // Navigation send mesg frame to MCU --------
    #define ID_CMD_TIME_SET        0x01
    #define ID_CMD_MOP_SET         0x02
    #define ID_CMD_JAW_SET         0x03
    #define ID_CMD_CATCHER_SET     0x04
    #define ID_CMD_MAGNET_SET      0x05
    #define ID_CMD_MECARM_SET      0x06
    #define ID_CMD_LIFTS_SET       0x07
    #define ID_CMD_LED_SET         0x08
    #define ID_CMD_HEARTBEAT       0x09
    /*---------------------------------------------*/
    #define  STORE_NUM_TIME_SET       0
    #define  STORE_NUM_MOP_SET        1
    #define  STORE_NUM_JAW_SET        2
    #define  STORE_NUM_CATCHER_SET    3
    #define  STORE_NUM_MAGNET_SET     4
    #define  STORE_NUM_MECARM_SET     5
    #define  STORE_NUM_LIFTS_SET      6
    #define  STROE_NUM_LED_SET        7
    /*---------------------------------------------*/
    #define DATA_LEGTH_TIME_SET        8
    #define DATA_LEGTH_MOP_SET         3
    #define DATA_LEGTH_JAW_SET         4
    #define DATA_LEGTH_CATCHER_SET     4
    #define DATA_LEGTH_MAGNET_SET      2   // 
    #define DATA_LEGTH_MECARM_SET      25
    #define DATA_LEGTH_LIFTS_SET       8
    #define DATA_LEGTH_LED_SET         2
    #define DATA_LEGTH_HEARTBEAT       0

    // MCU send mesg frame to Navigation --------
    #define ID_MEC_ARM_UPLOAD        0x11
    #define ID_LIFTS_UPLOAD          0x12
    #define ID_JAW_MOTOR_UPLOAD      0x13


    #define DATA_LEGTH_MEC_ARM       84
    #define DATA_LEGTH_LIFTS         28
    #define DATA_LEGTH_JAW_MOTOR     8
    
    typedef struct protocolOutputBuffer_TP
    {
        uint8_t buffer[WRITE_BUFFER_SIZE];
        uint16_t length;
    }protocolOutputBuffer_TP;
    
    /*this enum indicate all function code  according to our private protocol*/
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

        // 吸尘设置数据结构
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
                uint8_t     Motor_Control_Mode;
                uint8_t     reserver1;
                uint16_t     reserver2;                   
                uint32_t     position_motor1;   // 
                uint32_t     position_motor2;   //
                uint32_t     position_motor3;   //
                uint32_t     position_motor4;   //
                uint32_t     position_motor5;   //
                uint32_t     position_motor6;   //
                } real;
                uint8_t data[28];
            };
        };

        // 升降机构设置数据结构
        struct LiftsSet {
            union {
                struct {
                    uint32_t left_pos;
                    uint32_t right_pos;  // 预留
                } real;
                uint8_t data[8];
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
    struct UploadMecArm 
    {
        union{
            struct
            {
            uint16_t   reserve;
            uint16_t   time;
            
            float      pos_rad_motor1;    //Î»ÖÃ,×ª»»Îª¹€³Ìµ¥Î»
            float      vel_rad_s_motor1;  //ËÙ¶È,×ª»»Îª¹€³Ìµ¥Î»
            float      torque_nm_motor1;  //Å€ŸØ,×ª»»Îª¹€³Ìµ¥Î» 		
            uint16_t   status_motor1;     //×ŽÌ¬×Ö
            uint16_t   error_motor1;      //ŽíÎóÂë
                
            float      pos_rad_motor2;
            float      vel_rad_s_motor2; 
            float      torque_nm_motor2; 			
            uint16_t   status_motor2;     
            uint16_t   error_motor2;     
                
            float      pos_rad_motor3;
            float      vel_rad_s_motor3;
            float      torque_nm_motor3; 	
            uint16_t   status_motor3;    
            uint16_t   error_motor3;     		
                
            float      pos_rad_motor4; 
            float      vel_rad_s_motor4;
            float      torque_nm_motor4; 	
            uint16_t   status_motor4;    
            uint16_t   error_motor4;    	

            float      pos_rad_motor5;
            float      vel_rad_s_motor5;
            float      torque_nm_motor5; 	
            uint16_t   status_motor5;    
            uint16_t   error_motor5;    
                
            }real;
        
            uint8_t data[84];
        };
    };

        // 升降机构上传数据结构
        struct UploadLifts {
            union {
                struct {
                    u_int16_t reserver;
                    uint16_t time;
                    float positon_left;  // 预留
                    float speed_left;  // 预留
                    float current_left;  // 预留
                    float positon_right;  // 预留
                    float speed_right;  // 预留
                    float current_right;  // 预留
                } real;
                uint8_t data[28];
            };
        };

        // 夹爪上传数据结构
        struct UploadJaw {
            union {
                struct {
                    u_int16_t reserver;                  
                    uint16_t time;
                    float jaw_motor_angle;  // 0-180度
                } real;
                uint8_t data[8];
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
                void print() const {
                std::cout << "Frame Info: "
                  << "\n  Head: 0x" << std::hex << (int)head_H << " 0x" << (int)head_L
                  << "\n  FrameID: 0x" << (int)frame_ID_H << " 0x" << (int)frame_ID_L
                  << "\n  Length: " << std::dec << (int)length
                  << "\n  CmdID: 0x" << std::hex << (int)cmd_ID
                  << "\n  CRC: 0x" << (int)CRC_H << " 0x" << (int)CRC_L
                  << std::endl;
        
        // 打印数据内容（示例打印前16字节）
        std::cout << "  Data: ";
        for (int i = 0; i < std::min(84, (int)length-1); ++i) {
            std::cout << "0x" << std::hex << (int)databuf[i] << " ";
        }
        std::cout << std::endl;
    }
        };

    extern TimerHostComm Timer_HostComm;
    // extern HeartBeat HeartBeat;  // 注释掉的变量也需要声明

    // ----------- MCU->导航的数据结构 -----------
    extern UploadMecArm commmecarm;
    extern UploadLifts commlifts;
    extern UploadJaw commjaw;

    // ----------- MCU->orin的通信帧 -----------
    extern CommFrame uploadframeMecarm;
    extern CommFrame uploadframeLifts;
    extern CommFrame uploadframeJaw;

    // ----------- orin->MCU的数据结构 -----------
    extern TimeSet dataTimeSet;
    extern MopSet dataMopSet;
    extern JawSet dataJawSet;
    extern CatcherSet dataCatcherSet;
    extern MagnetSet dataMagnetSet;
    extern MecArmSet dataMecArmSet;
    extern LiftsSet dataLiftsSet;
    extern LedSet dataLedSet;
    extern Heartbeat dataHeartbeat;

    // ----------- orin->MCU的通信帧 -----------
    extern CommFrame cmdframTimeSet;
    extern CommFrame cmdframMopSet;
    extern CommFrame cmdframJawSet;
    extern CommFrame cmdframCatcherSet;
    extern CommFrame cmdframMagnetet;
    extern CommFrame cmdframMecArmSet;
    extern CommFrame cmdframLiftsSet;
    extern CommFrame cmdframLedSet;
    // extern CommFrame cmdframHeartbeat;
    extern void  initializeFrames();
    // ----------- 命令帧组 -----------
    extern CommFrame* cmdframGroup[8];
    extern CommFrame* statusframGroup[3];  

    }
#endif 