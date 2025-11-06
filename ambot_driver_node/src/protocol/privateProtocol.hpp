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
#include <filter.hpp> // 包含 memcpy
namespace bimax_driver_ns
{

    class PrivateProtocolCLASS
    {
    private:
        std::shared_ptr<bimax_driver_ns::RosClass> ros;
        std::function<void(float)> jaw_position_callback_;
    public:
        // 为每个电机的每个参数创建组合滤波器
        struct MotorFilters {
            MedianClipFilter pos_filter;    // 位置滤波器
            MedianClipFilter vel_filter;    // 速度滤波器  
            MedianClipFilter torque_filter; // 力矩滤波器
            
            // 构造函数：设置不同的窗口大小和限幅阈值
            MotorFilters() 
                : pos_filter(3, 1.0f),      // 位置：窗口7，最大变化0.5rad
                vel_filter(3, 2.0f),     // 速度：窗口5，最大变化2.0rad/s
                torque_filter(3, 4.0f)    // 力矩：窗口5，最大变化5.0Nm
            {}
        };
        std::array<MotorFilters, 5> motor_filters_;  // 5个电机的滤波器
        PrivateProtocolCLASS(const std::shared_ptr<RosClass>& ros);
        ~PrivateProtocolCLASS();
        void lifterDateUpdate(std::vector<uint8_t> data, float& left_value, float& right_value);
        void jawMotorDateUpdate(std::vector<uint8_t> data, float& postion);
        void yiyouMotorDateUpdate(std::vector<uint8_t> data, YiyouMecArm& mecarm); 
        void updateDataConsumer(YiyouMecArm &mecarm,float &lifter_l_pos,float  &lifter_r_pos,float &jaw_pos);
        void setJawPositionCallback(std::function<void(float)> callback) {jaw_position_callback_ = callback;}
        uint8_t  commFrameStore(CommFrame* statusframGroup, const uint8_t* databuf);
        uint8_t  commFrameUpload(CommFrame* frame, uint8_t* output_buf) ;
        uint16_t processFrame(const uint8_t* data, uint16_t length);
        uint16_t getExpectedFrameLength(uint16_t cmdId) const;
    };


}

#endif
