/*  A class of private protocol between PC and MCU

*/

#include "privateProtocol.hpp"

namespace bimax_driver_ns
{
        UnifiedDeviceQueue arm_queue; // 统一队列替换原有三个队列
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

    PrivateProtocolCLASS::PrivateProtocolCLASS(const std::shared_ptr<RosClass>& ros) : ros(ros)
    {
         RCLCPP_INFO(ros->get_logger(), "PrivateProtocolCLASS Interface init successfully!"); 
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
    uint16_t PrivateProtocolCLASS::getExpectedFrameLength(uint16_t cmdId) const {
        // 根据您的协议返回期望的帧长度
        switch (cmdId) {
            case ID_MEC_ARM_UPLOAD: return 85; // 示例值，根据实际协议修改
            case ID_LIFTS_UPLOAD: return 29;   // 示例值
            case ID_JAW_MOTOR_UPLOAD: return 9; // 示例值
            default: return 9; // 最小帧长度
        }
    }
        // 帧处理主函数
    uint16_t PrivateProtocolCLASS::processFrame(const uint8_t* data, uint16_t length) {
        uint16_t processed_bytes = 0; // 实际处理了的字节数
        uint16_t frames_found = 0;
        
        for (uint16_t i = 0; i < length; ) { // 注意：这里没有i++
            // 检查剩余数据是否足够寻找帧头
            if (i + 1 >= length) break;
            
            // 查找帧头
            if (data[i] == FRAME_HEAD_H && data[i + 1] == FRAME_HEAD_L) {
                // 检查是否能够读取命令码
                if (i + 5 >= length) {
                    // 数据不完整，等待更多数据
                    break;
                }
                
                uint16_t cmdId = data[i + 5];
                uint16_t store_length = 0;
                DeviceType type;
                
                // 根据命令码获取期望的完整帧长度
                uint16_t expected_length = getExpectedFrameLength(cmdId);
                
                // 检查是否有完整的帧数据
                if (i + expected_length > length) {
                    // 数据不完整，等待更多数据
                    break;
                }
                
                switch (cmdId) {
                    case ID_MEC_ARM_UPLOAD:
                        type = DeviceType::MEC_ARM;
                        store_length = commFrameStore(statusframGroup[ID_MEC_ARM_STORE], &data[i]);
                        if (store_length > 0 && store_length <= expected_length) {
                            arm_queue.push(type, statusframGroup[ID_MEC_ARM_STORE]->databuf, store_length-1);
                            #ifdef DEBUG_MODE
                            statusframGroup[ID_MEC_ARM_STORE]->print();  
                            #endif

                            frames_found++;
                        }
                        break;
                        
                    case ID_LIFTS_UPLOAD:
                        type = DeviceType::LIFTS_MOTOR;
                        store_length = commFrameStore(statusframGroup[ID_LIFTS_STORE], &data[i]);
                        if (store_length > 0 && store_length <= expected_length) {
                            arm_queue.push(type, statusframGroup[ID_LIFTS_STORE]->databuf, store_length-1);
                            #ifdef DEBUG_MODE
                            statusframGroup[ID_LIFTS_STORE]->print();                      
                            #endif
                            frames_found++;
                        }
                        break;
                        
                    case ID_JAW_MOTOR_UPLOAD:
                        type = DeviceType::JAW_MOTOR;
                        store_length = commFrameStore(statusframGroup[ID_JAW_MOTOR_STORE], &data[i]);
                        if (store_length > 0 && store_length <= expected_length) {
                            arm_queue.push(type, statusframGroup[ID_JAW_MOTOR_STORE]->databuf, store_length-1);
                            #ifdef DEBUG_MODE
                            statusframGroup[ID_JAW_MOTOR_STORE]->print();                      
                            #endif
                            frames_found++;
                        }
                        break;
                        
                    default:
                        // 未知命令码，跳过这个帧头继续搜索
                        i++;
                        continue;
                }
                
                if (store_length > 0) {
                    // 成功处理一帧，跳到下一帧开始位置
                    i += store_length;
                    processed_bytes = i; // 更新已处理字节数
                } else {
                    // 解析失败，跳过这个帧头继续搜索
                    i++;
                }
            } else {
                // 不是帧头，继续搜索
                i++;
            }
            
            // 安全限制：单次处理不要超过缓冲区大小
            if (processed_bytes > length) {
                break;
            }
        }
        
        return processed_bytes; // 返回实际处理了多少字节
    }

    void  PrivateProtocolCLASS::updateDataConsumer(YiyouMecArm &mecarm,float &lifterLeftPos,float  &lifterRightPos,float &jawPos) {
            DeviceType type;
            std::vector<uint8_t> data;
            if (arm_queue.pop(type,data, 1ms)) {
                switch (type) {
                    case DeviceType::MEC_ARM:{
                        yiyouMotorDateUpdate(data,mecarm); 
                        break;}
                    case DeviceType::LIFTS_MOTOR:{
                        lifterDateUpdate(data, lifterLeftPos, lifterRightPos); // 通过引用返回
                        break;}
                    case DeviceType::JAW_MOTOR:{
                        jawMotorDateUpdate(data, jawPos);
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
 // Motor1 - 应用中值+限幅组合滤波
        mecarm.pos_rad_motor1 = motor_filters_[0].pos_filter.filter(readFloat(offset)); offset += 4;
        mecarm.vel_rad_s_motor1 = motor_filters_[0].vel_filter.filter(readFloat(offset)); offset += 4;
        mecarm.torque_nm_motor1 = motor_filters_[0].torque_filter.filter(readFloat(offset)); offset += 4;
        mecarm.status_motor1 = readUint16(offset); offset += 2;
        mecarm.error_motor1 = readUint16(offset); offset += 2;

        // Motor2
        mecarm.pos_rad_motor2 = motor_filters_[1].pos_filter.filter(readFloat(offset)); offset += 4;
        mecarm.vel_rad_s_motor2 = motor_filters_[1].vel_filter.filter(readFloat(offset)); offset += 4;
        mecarm.torque_nm_motor2 = motor_filters_[1].torque_filter.filter(readFloat(offset)); offset += 4;
        mecarm.status_motor2 = readUint16(offset); offset += 2;
        mecarm.error_motor2 = readUint16(offset); offset += 2;

        // Motor3
        mecarm.pos_rad_motor3 = motor_filters_[2].pos_filter.filter(readFloat(offset)); offset += 4;
        mecarm.vel_rad_s_motor3 = motor_filters_[2].vel_filter.filter(readFloat(offset)); offset += 4;
        mecarm.torque_nm_motor3 = motor_filters_[2].torque_filter.filter(readFloat(offset)); offset += 4;
        mecarm.status_motor3 = readUint16(offset); offset += 2;
        mecarm.error_motor3 = readUint16(offset); offset += 2;

        // Motor4
        mecarm.pos_rad_motor4 = motor_filters_[3].pos_filter.filter(readFloat(offset)); offset += 4;
        mecarm.vel_rad_s_motor4 = motor_filters_[3].vel_filter.filter(readFloat(offset)); offset += 4;
        mecarm.torque_nm_motor4 = motor_filters_[3].torque_filter.filter(readFloat(offset)); offset += 4;
        mecarm.status_motor4 = readUint16(offset); offset += 2;
        mecarm.error_motor4 = readUint16(offset); offset += 2;

        // Motor5
        mecarm.pos_rad_motor5 = motor_filters_[4].pos_filter.filter(readFloat(offset)); offset += 4;
        mecarm.vel_rad_s_motor5 = motor_filters_[4].vel_filter.filter(readFloat(offset)); offset += 4;
        mecarm.torque_nm_motor5 = motor_filters_[4].torque_filter.filter(readFloat(offset)); offset += 4;
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
    }

    void PrivateProtocolCLASS::lifterDateUpdate(std::vector<uint8_t> data, float& leftValue, float& rightValue) {
        uint8_t leftBytes[] = { data[4], data[5], data[6], data[7] };
        memcpy(&leftValue, leftBytes, sizeof(float));

        uint8_t rightBytes[] = { data[16], data[17], data[18], data[19] };
        memcpy(&rightValue, rightBytes, sizeof(float));
    }
    /**  
    *   @brief      create get ID protocol frame
        Parameters:
    *   @param      output  [out]the frame buffer wait to send
    *   @return     none
        */

    uint8_t PrivateProtocolCLASS::commFrameStore(CommFrame* statusframGroup, const uint8_t* databuf) 
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

    uint8_t PrivateProtocolCLASS::commFrameUpload(CommFrame* frame, uint8_t* output_buf) {
        // 参数检查
        if (!frame || !frame->databuf || !output_buf || frame->length < 1 || frame->length > 256) {
            RCLCPP_WARN(ros->get_logger(), "commFrameUpload frame is warnning!");    
            return 0;  // 或抛出异常
        }

        // 线程安全的计数器
        size_t total_bytes=0;
        uint16_t crc_value=0;
        uint8_t databuf_temp[256] = {0};  
        std::stringstream hex_stream;
        static std::atomic<uint16_t> upload_cnt{0};
        const uint16_t frame_id = ++upload_cnt;
        const uint16_t data_length = frame->length - 1;
        frame->head_H = static_cast<uint8_t>(0xF0);
        frame->head_L = static_cast<uint8_t>(0XF0);
        // 设置Frame ID
        frame->frame_ID_H = static_cast<uint8_t>((frame_id >> 8) & 0xFF);
        frame->frame_ID_L = static_cast<uint8_t>(frame_id & 0xFF);

        databuf_temp[0] = frame->cmd_ID;
        for (uint8_t i = 1; i <= data_length; i++) {
            databuf_temp[i] = frame->databuf[i-1];
        }

        // 计算CRC

        crc_value = usMBCRC16(databuf_temp, frame->length);
        frame->CRC_H = static_cast<uint8_t>(crc_value >> 8);
        frame->CRC_L = static_cast<uint8_t>(crc_value & 0xFF);

        // 合并数据到output_buf
        memcpy(output_buf, frame, 6);                               // 帧头
        memcpy(output_buf + 6, frame->databuf, data_length);        // 数据
        memcpy(output_buf + 6 + data_length, &frame->CRC_H, 1);     // CRC高字节
        memcpy(output_buf + 6 + data_length + 1, &frame->CRC_L, 1); // CRC低字节
        total_bytes=6 + data_length + 2;
        // Build hex string for logging

        for (size_t i = 0; i < total_bytes; i++) {
            hex_stream << std::hex << std::setw(2) << std::setfill('0') 
                    << static_cast<int>(output_buf[i]) << " ";
        }
        RCLCPP_INFO(ros->get_logger(), "Output frame: %s", hex_stream.str().c_str());
        return total_bytes; 
    }

}