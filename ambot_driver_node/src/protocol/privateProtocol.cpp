/*  A class of private protocol between PC and MCU

*/

#include "privateProtocol.hpp"

namespace bimax_driver_ns
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
        MecArmMitSet dataArmMitset={};
        // 导航发送给MCU的帧结构初始化
        CommFrame cmdframTimeSet = {
            .head_H =FRAME_HEAD_H ,
            .head_L = FRAME_HEAD_L,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LEGTH_TIME_SET,
            .cmd_ID =  ID_CMD_TIME_SET,
            .databuf = dataTimeSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframMopSet = {
            .head_H =FRAME_HEAD_H ,
            .head_L = FRAME_HEAD_L,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LEGTH_MOP_SET,
            .cmd_ID =  ID_CMD_MOP_SET,
            .databuf = dataMopSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframJawSet = {
            .head_H =FRAME_HEAD_H ,
            .head_L = FRAME_HEAD_L,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LEGTH_JAW_SET,
            .cmd_ID =  ID_CMD_JAW_SET,
            .databuf = dataJawSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframCatcherSet = {
            .head_H =FRAME_HEAD_H ,
            .head_L = FRAME_HEAD_L,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LEGTH_CATCHER_SET,
            .cmd_ID =  ID_CMD_CATCHER_SET,
            .databuf = dataCatcherSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframMagnetet = {
            .head_H =FRAME_HEAD_H ,
            .head_L = FRAME_HEAD_L,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LEGTH_MAGNET_SET,
            .cmd_ID =  ID_CMD_MAGNET_SET,
            .databuf = dataMagnetSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframMecArmSet = {
            .head_H =FRAME_HEAD_H ,
            .head_L = FRAME_HEAD_L,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LEGTH_MECARM_SET,
            .cmd_ID =  ID_CMD_MECARM_SET,
            .databuf = dataMecArmSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };
        CommFrame cmdArmMitSet = {
            .head_H =FRAME_HEAD_H ,
            .head_L = FRAME_HEAD_L,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LEGTH_MECARM_MIT_SET,
            .cmd_ID =  ID_CMD_MECARM_MIT_SET,
            .databuf = dataArmMitset.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframLiftsSet = {
            .head_H =FRAME_HEAD_H ,
            .head_L = FRAME_HEAD_L,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LEGTH_LIFTS_SET,
            .cmd_ID =  ID_CMD_LIFTS_SET,
            .databuf = dataLiftsSet.data,
            .CRC_H = 0,
            .CRC_L = 0
        };

        CommFrame cmdframLedSet = {
            .head_H =FRAME_HEAD_H ,
            .head_L = FRAME_HEAD_L,
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
        CommFrame* cmdframGroup[9] = {
            &cmdframTimeSet,
            &cmdframMopSet,
            &cmdframJawSet,
            &cmdframCatcherSet,
            &cmdframMagnetet,
            &cmdframMecArmSet,
            &cmdframLiftsSet,
            &cmdframLedSet,
            &cmdArmMitSet
        };    

    PrivateProtocolCLASS::PrivateProtocolCLASS(const std::shared_ptr<RosClass>& ros) : ros(ros)
    {

         RCLCPP_INFO(ros->get_logger(), "PrivateProtocolCLASS Interface init successfully!"); 
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

    void  PrivateProtocolCLASS::updateDataConsumer(YiyouMecArm &mecarm,float &lifter_l_pos,float  &lifter_r_pos,float &jaw_pos) {
            DeviceType type;
            std::vector<uint8_t> data;
            if (arm_queue.pop(type,data, 2ms)) {
                switch (type) {
                    case DeviceType::MEC_ARM:{
                        yiyouMotorDateUpdate(data,mecarm); 
                        break;}
                    case DeviceType::LIFTS:{
                        lifterDateUpdate(data, lifter_l_pos, lifter_r_pos); // 通过引用返回
                        break;}
                    case DeviceType::JAW_MOTOR:{
                        jawMotorDateUpdate(data, jaw_pos);
                        break;}
                }
            }
        }
    void PrivateProtocolCLASS::yiyouMotorDateUpdate(std::vector<uint8_t> data, YiyouMecArm& mecarm) 
    {
        // 检查数据大小是否足够（至少需要 15个float × 4字节 + 10个uint16_t × 2字节 = 80字节）
        if (data.size() < 84) {
            RCLCPP_WARN(ros->get_logger(), "yiyouMotorDateUpdate date size <84!"); 
            return;
        }

        // 辅助函数：从 data 的指定位置解析 float
        auto readFloat = [&data](size_t offset) -> float {
            float value;
            memcpy(&value, &data[offset], sizeof(float));
            return value;
        };

        // 辅助函数：从 data 的指定位置解析 uint16_t
        auto readUint16 = [&data](size_t offset) -> uint16_t {
            uint16_t value;
            memcpy(&value, &data[offset], sizeof(uint16_t));
            return value;
        };

        // 解析并赋值 motor1
        size_t offset = 4;
        mecarm.pos_rad_motor1 = readFloat(offset); offset += 4;
        mecarm.vel_rad_s_motor1 = readFloat(offset); offset += 4;
        mecarm.torque_nm_motor1 = readFloat(offset); offset += 4;
        mecarm.status_motor1 = readUint16(offset); offset += 2;
        mecarm.error_motor1 = readUint16(offset); offset += 2;

        // 解析并赋值 motor2
        mecarm.pos_rad_motor2 = readFloat(offset); offset += 4;
        mecarm.vel_rad_s_motor2 = readFloat(offset); offset += 4;
        mecarm.torque_nm_motor2 = readFloat(offset); offset += 4;
        mecarm.status_motor2 = readUint16(offset); offset += 2;
        mecarm.error_motor2 = readUint16(offset); offset += 2;

        // 解析并赋值 motor3
        mecarm.pos_rad_motor3 = readFloat(offset); offset += 4;
        mecarm.vel_rad_s_motor3 = readFloat(offset); offset += 4;
        mecarm.torque_nm_motor3 = readFloat(offset); offset += 4;
        mecarm.status_motor3 = readUint16(offset); offset += 2;
        mecarm.error_motor3 = readUint16(offset); offset += 2;

        // 解析并赋值 motor4
        mecarm.pos_rad_motor4 = readFloat(offset); offset += 4;
        mecarm.vel_rad_s_motor4 = readFloat(offset); offset += 4;
        mecarm.torque_nm_motor4 = readFloat(offset); offset += 4;
        mecarm.status_motor4 = readUint16(offset); offset += 2;
        mecarm.error_motor4 = readUint16(offset); offset += 2;

        // 解析并赋值 motor5
        mecarm.pos_rad_motor5 = readFloat(offset); offset += 4;
        mecarm.vel_rad_s_motor5 = readFloat(offset); offset += 4;
        mecarm.torque_nm_motor5 = readFloat(offset); offset += 4;
        mecarm.status_motor5 = readUint16(offset); offset += 2;
        mecarm.error_motor5 = readUint16(offset); offset += 2;

        // 打印调试信息（可选）
        // std::cout << "Motor1 Position: " << mecarm.pos_rad_motor1 << " rad" << std::endl;
        // std::cout << "Motor2 Position: " << mecarm.pos_rad_motor2 << " rad" << std::endl;
        // std::cout << "Motor3 Position: " << mecarm.pos_rad_motor3 << " rad" << std::endl;
        // std::cout << "Motor4 Position: " << mecarm.pos_rad_motor4 << " rad" << std::endl;
        // std::cout << "Motor5 Position: " << mecarm.pos_rad_motor5 << " rad" << std::endl;
    }
   
    void PrivateProtocolCLASS::jawMotorDateUpdate(std::vector<uint8_t> data, float& postion) 
    {
        uint8_t pos[] = { data[4], data[5], data[6], data[7] };
        memcpy(&postion, pos, sizeof(float));
        // std::cout << "夹爪位置: " << postion << std::endl;
    }

    void PrivateProtocolCLASS::lifterDateUpdate(std::vector<uint8_t> data, float& left_value, float& right_value) {
        uint8_t left_bytes[] = { data[4], data[5], data[6], data[7] };
        memcpy(&left_value, left_bytes, sizeof(float));

        uint8_t right_bytes[] = { data[16], data[17], data[18], data[19] };
        memcpy(&right_value, right_bytes, sizeof(float));

        // std::cout << "左升降位置: " << left_value << std::endl;
        // std::cout << "右升降位置: " << right_value << std::endl;
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

    uint8_t PrivateProtocolCLASS::comm_frame_upload(CommFrame* frame, uint8_t* output_buf) {
        // 参数检查
        if (!frame || !frame->databuf || !output_buf || frame->length < 1 || frame->length > 256) {
            RCLCPP_WARN(ros->get_logger(), "comm_frame_upload frame is warnning!");    
            return 0;  // 或抛出异常
        }

        // 线程安全的计数器
        static std::atomic<uint16_t> upload_cnt{0};
        const uint16_t frame_id = ++upload_cnt;
        const uint16_t data_length = frame->length - 1;
        frame->head_H = static_cast<uint8_t>(0xF0);
        frame->head_L = static_cast<uint8_t>(0XF0);
        // 设置Frame ID
        frame->frame_ID_H = static_cast<uint8_t>((frame_id >> 8) & 0xFF);
        frame->frame_ID_L = static_cast<uint8_t>(frame_id & 0xFF);
        // u_int8_t arr[]={0xC8 ,0xCC, 0x4C, 0x3D, 0xC8 ,0xCC, 0x4C, 0x3D};
        // frame->databuf=arr;
        // 准备CRC计算数据
        uint8_t databuf_temp[256] = {0};
        databuf_temp[0] = frame->cmd_ID;
        for (uint8_t i = 1; i <= data_length; i++) {
            databuf_temp[i] = frame->databuf[i-1];
        }

        // 计算CRC
        uint16_t crc_value = usMBCRC16(databuf_temp, frame->length);
        frame->CRC_H = static_cast<uint8_t>(crc_value >> 8);
        frame->CRC_L = static_cast<uint8_t>(crc_value & 0xFF);

        // 合并数据到output_buf
        memcpy(output_buf, frame, 6);                               // 帧头
        memcpy(output_buf + 6, frame->databuf, data_length);        // 数据
        memcpy(output_buf + 6 + data_length, &frame->CRC_H, 1);     // CRC高字节
        memcpy(output_buf + 6 + data_length + 1, &frame->CRC_L, 1); // CRC低字节
        size_t total_bytes=6 + data_length + 2;
        // Build hex string for logging
        std::stringstream hex_stream;
        for (size_t i = 0; i < total_bytes; i++) {
            hex_stream << std::hex << std::setw(2) << std::setfill('0') 
                    << static_cast<int>(output_buf[i]) << " ";
        }

        RCLCPP_INFO(ros->get_logger(), "Output frame: %s", hex_stream.str().c_str());
        return total_bytes; 
    }
    // void PrivateProtocolCLASS::createCommandFrame(const uint8_t functionCode, const uint8_t commandNum, const protocolInputBuffer_TP& in, protocolOutputBuffer_TP &output)
    // {

    // }

}