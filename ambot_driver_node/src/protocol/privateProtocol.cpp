/*  A class of private protocol between PC and MCU
    *
    * Author： chen chen
    * Date: 2024-8-22
    * Email: 1240563221@qq.com
    *
*/

#include "privateProtocol.hpp"

namespace ambot_driver_ns
{
        UnifiedDeviceQueue arm_queue; // 统一队列替换原有三个队列
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
        CommFrame* statusframGroup[3] = {
            &uploadframeMecarm,
            &uploadframeLifts,
            &uploadframeJaw
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

    PrivateProtocolCLASS::PrivateProtocolCLASS(/* args */)
    {

        
        // motorLimit = {4*MY_PI, -4*MY_PI, 30, -30, 12, -12, 500., 0, 5., 0};
    }
    
    PrivateProtocolCLASS::~PrivateProtocolCLASS()
    {
    }
        uint16_t isFrameHeaderValid(const uint8_t* data, uint16_t index)  {
        if((data[index] == FRAME_HEAD_H) && (data[index + 1] == FRAME_HEAD_L))
            return index;
        else 
            return -1;
    }
        // 帧处理主函数
    uint8_t PrivateProtocolCLASS::processFrame(const uint8_t* data, uint16_t length) {
        uint8_t result = 0; // 初始化计数器
        uint16_t  store_length = 0; //读到的当前帧长度  
        uint16_t cmdId =0;
        for (uint16_t i = 0; i < length; i++) {
            if ((data[i]==FRAME_HEAD_H)&& (data[i + 1] == FRAME_HEAD_L)) {
                store_length = 0;
                cmdId = data[i + 5]; // 命令码ID 
                DeviceType type;
                switch (cmdId) {
                    case ID_MEC_ARM_UPLOAD:
                        type = DeviceType::MEC_ARM;
                        store_length = comm_frame_store(statusframGroup[ID_MEC_ARM_STORE], &data[i]);
                        arm_queue.push(type, statusframGroup[ID_MEC_ARM_STORE]->databuf,store_length-1);
                        #ifdef DEBUG_MODE
                        statusframGroup[ID_MEC_ARM_STORE]->print();  
                        #endif
                        break;
                        
                    case ID_LIFTS_UPLOAD:
                         type = DeviceType::LIFTS;
                        store_length = comm_frame_store(statusframGroup[ID_LIFTS_STORE], &data[i]);
                        arm_queue.push(type, statusframGroup[ID_LIFTS_STORE]->databuf,store_length-1);
                        #ifdef DEBUG_MODE
                        statusframGroup[ID_LIFTS_STORE]->print();                      
                        #endif                        
                        break;
                        
                    case ID_JAW_MOTOR_UPLOAD:
                        type = DeviceType::JAW_MOTOR;
                        store_length = comm_frame_store(statusframGroup[ID_JAW_MOTOR_STORE], &data[i]);
                         arm_queue.push(type, statusframGroup[ID_JAW_MOTOR_STORE]->databuf,store_length-1);
                        #ifdef DEBUG_MODE
                        statusframGroup[ID_JAW_MOTOR_STORE]->print();                      
                        #endif   
                        break;
                }
                
                if (store_length != 0) {
                    result++;
                }
                i = i + store_length; //跳出for，找下一帧
            }
        }
        
        
        return result;
    }

        void  PrivateProtocolCLASS::data_consumer() {
            DeviceType type;
            std::vector<uint8_t> data;
            if (arm_queue.pop(type,data, 10ms)) {
                switch (type) {
                    case DeviceType::MEC_ARM:
                        std::cout<<"MEC_ARM"<<std::endl;
                        break;
                    case DeviceType::LIFTS:
                        std::cout<<"LIFTS"<<std::endl;
                        break;
                    case DeviceType::JAW_MOTOR:
                        std::cout<<"JAW_MOTOR"<<std::endl;
                        break;
                }
            }
        }
    /**  
    *   @brief      create get ID protocol frame
        Parameters:
    *   @param      output  [out]the frame buffer wait to send
    *   @return     none
        */

    uint8_t PrivateProtocolCLASS::comm_frame_store(CommFrame* statusframGroup, const uint8_t* databuf) 
    {
         uint8_t store_length = 0;  // store_length=事件控制码长度(1byte)+数据内容的长度
         uint16_t data_length = statusframGroup->length - 1;  // data_length=store_length-事件控制码长度(1byte)
        
         statusframGroup->CRC_H = databuf[6 + data_length + 0];
         statusframGroup->CRC_L = databuf[6 + data_length + 1];  
        // std::cout << "CRC_H"<<std::hex << (int)databuf[6 + data_length + 0]<<std::endl;
        // std::cout << "CRC_L"<<std::hex << (int)databuf[6 + data_length + 1]<<std::endl;
        uint16_t crc_rslt =  usMBCRC16(const_cast<uint8_t*>(&databuf[5]), (data_length + 1));
        
        if(((statusframGroup->CRC_H << 8) | (statusframGroup->CRC_L)) == crc_rslt)
        {
            statusframGroup->head_H     = databuf[0];
            statusframGroup->head_L     = databuf[1];
            statusframGroup->frame_ID_H = databuf[2];
            statusframGroup->frame_ID_L = databuf[3];
            statusframGroup->cmd_ID     = databuf[5];
            
            memcpy(statusframGroup->databuf, &databuf[6], data_length);
            
            store_length = statusframGroup->length;
        }
        else
        {
            store_length = 0;
        }
        
        return store_length;
    }
    // void PrivateProtocolCLASS::createCommandFrame(const uint8_t functionCode, const uint8_t commandNum, const protocolInputBuffer_TP& in, protocolOutputBuffer_TP &output)
    // {

    // }

}