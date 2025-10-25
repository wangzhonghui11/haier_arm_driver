/*  A class of private protocol between PC and MCU
    *
    * Author： chen chen
    * Date: 2024-8-22
    * Email: 1240563221@qq.com
    *
*/

#ifndef __PRIVATE_PROTOCOL_HPP__
#define __PRIVATE_PROTOCOL_HPP__
#include <atomic>
#include <iostream>
#include <vector>
#include <string.h>
#include "protocolStruct.hpp"
#include "ambotRosClass.hpp"
#include "crcModbus.hpp"
#include "queue.hpp"
#include <iomanip>  // 用于 std::hex 格式化
#include <cstring> // 包含 memcpy
namespace bimax_driver_ns
{

    class PrivateProtocolCLASS
    {
    private:
        
        std::shared_ptr<bimax_driver_ns::RosClass> ros;
    public:
        PrivateProtocolCLASS(const std::shared_ptr<RosClass>& ros);
        ~PrivateProtocolCLASS();
        uint8_t processFrame(const uint8_t* data, uint16_t length);
        uint8_t comm_frame_store(CommFrame* statusframGroup, const uint8_t* databuf);
        //void createCommandFrame(const uint8_t functionCode, const uint8_t commandNum, const protocolInputBuffer_TP& in, protocolOutputBuffer_TP &output);
        uint8_t comm_frame_upload(CommFrame* frame, uint8_t* output_buf) ;
        void lifterDateUpdate(std::vector<uint8_t> data, float& left_value, float& right_value);
        void jawMotorDateUpdate(std::vector<uint8_t> data, float& postion);
        void yiyouMotorDateUpdate(std::vector<uint8_t> data, YiyouMecArm& mecarm); 
        void updateDataConsumer(YiyouMecArm &mecarm,float &lifter_l_pos,float  &lifter_r_pos,float &jaw_pos);
    };


}

#endif
