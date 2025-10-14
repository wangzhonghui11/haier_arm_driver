#include "protocol_struct.hpp"  // 包含对应的头文件

namespace ambot_driver_ns {
    // 定时器和心跳结构体初始化
        TimerHostComm Timer_HostComm = {};
        // HeartBeat HeartBeat = {};

        /*****************************************************/ 
        // MCU发送给导航的数据结构初始化
        UploadMecArm commmecarm = {};
        UploadLifts commlifts = {};
        UploadJaw commjaw = {};

        // MCU发送给orin的帧结构初始化
        CommFrame uploadframeMecarm = {
            .head_H = FRAME_HEAD_H,
            .head_L = FRAME_HEAD_L,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LEGTH_MEC_ARM,
            .cmd_ID = ID_MEC_ARM_UPLOAD,
            .databuf = commmecarm.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame uploadframeLifts = {
            .head_H = FRAME_HEAD_H,
            .head_L = FRAME_HEAD_L,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LEGTH_LIFTS,
            .cmd_ID = ID_LIFTS_UPLOAD,
            .databuf = commlifts.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame uploadframeJaw = {
            .head_H = FRAME_HEAD_H,
            .head_L = FRAME_HEAD_L,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LEGTH_JAW_MOTOR,
            .cmd_ID =  ID_JAW_MOTOR_UPLOAD,
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
            .length = 1 + DATA_LEGTH_TIME_SET,
            .cmd_ID =  ID_CMD_TIME_SET,
            .databuf = dataTimeSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframMopSet = {
            .head_H = 0,
            .head_L = 0,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LEGTH_MOP_SET,
            .cmd_ID =  ID_CMD_MOP_SET,
            .databuf = dataMopSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframJawSet = {
            .head_H = 0,
            .head_L = 0,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LEGTH_JAW_SET,
            .cmd_ID =  ID_CMD_JAW_SET,
            .databuf = dataJawSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframCatcherSet = {
            .head_H = 0,
            .head_L = 0,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LEGTH_CATCHER_SET,
            .cmd_ID =  ID_CMD_CATCHER_SET,
            .databuf = dataCatcherSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframMagnetet = {
            .head_H = 0,
            .head_L = 0,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LEGTH_MAGNET_SET,
            .cmd_ID =  ID_CMD_MAGNET_SET,
            .databuf = dataMagnetSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframMecArmSet = {
            .head_H = 0,
            .head_L = 0,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LEGTH_MECARM_SET,
            .cmd_ID =  ID_CMD_MECARM_SET,
            .databuf = &dataMecArmSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframLiftsSet = {
            .head_H = 0,
            .head_L = 0,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LEGTH_LIFTS_SET,
            .cmd_ID =  ID_CMD_LIFTS_SET,
            .databuf = &dataLiftsSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframLedSet = {
            .head_H = 0,
            .head_L = 0,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LEGTH_LED_SET,
            .cmd_ID =  ID_CMD_LED_SET,
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