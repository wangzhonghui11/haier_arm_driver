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
#include "crcModbus.hpp"
#include "queue.hpp"
#include <iomanip>  // 用于 std::hex 格式化

namespace ambot_driver_ns
{

    class PrivateProtocolCLASS
    {
    private:


    public:
        PrivateProtocolCLASS(/* args */);
        ~PrivateProtocolCLASS();
        uint8_t processFrame(const uint8_t* data, uint16_t length);
        uint8_t comm_frame_store(CommFrame* statusframGroup, const uint8_t* databuf);
        void  data_consumer(); 
        //void createCommandFrame(const uint8_t functionCode, const uint8_t commandNum, const protocolInputBuffer_TP& in, protocolOutputBuffer_TP &output);
        uint8_t comm_frame_upload(CommFrame* frame, uint8_t* output_buf) ;
        
    };

    /**  
    *   @brief      the data transfer function: float to byte
        Parameters:
    *   @param      value    	[in]the reference of goal value 
    *   @param      buffer 		[in]the pointer of wait convert 
    *   @return     none
        */
    // template<typename T> void PrivateProtocolCLASS::otherTransferToByte(const T value, uint8_t* buffer)
    // {
    //     T temp = value;
    //     uint8_t* point = (uint8_t*)&temp;
    //     for (int i = 0; i < sizeof(T); i++)
    //         buffer[i] = *(point+i);
    // }

    // /**  
    // *   @brief      the data transfer function: float to byte
    //     Parameters:
    // *   @param      goalValue    	[in]the reference of goal value 
    // *   @param      buffer 		    [in]the pointer of wait convert 
    // *   @return     none
    //     */
    // template<typename T> void PrivateProtocolCLASS::byteTransferToOther(T& goalValue, const uint8_t* buffer)
    // {
    //     T temp;
    //     uint8_t* point = (uint8_t*)&temp;
    //     for (int i = 0; i < sizeof(T); i++)
    //         *(point + i) = *(buffer + i);
    //     goalValue = temp;
    // }

    // /**  
    // *   @brief      create command data frame for control command
    //     Parameters:
    // *   @param      functionCode    [in]the function code
    // *   @param      commandNum    	[in]the command code
    // *   @param      data 			[in]the input data
    // *   @param      output 			[out]the data wait to build/create 
    // *   @return     none
    //     */
    // template<typename T>
    // void PrivateProtocolCLASS::createCommandFrame(const uint8_t functionCode, const uint8_t commandNum, const std::vector<int> &ID, const std::vector<T>& data, protocolOutputBuffer_TP &output)
    // {
    //     uint8_t number =  data.size();
    //     memset(output.buffer, 0, WRITE_BUFFER_SIZE);

    //     output.buffer[ControlIndex.FRAME_HEAD] = FRAME_HEAD_DATA;
    //     output.buffer[ControlIndex.FUNCTION_CODE] = functionCode;
    //     output.buffer[ControlIndex.COMMAND_CODE] = commandNum;
    //     output.buffer[ControlIndex.MOTOR_NUM] = number;
    //     output.length = ControlIndex.DATA_LENGTH;
    //     for (int i = 0; i < number; i++)
    //     {
    //         output.buffer[ControlIndex.MOTOR_ID + ControlIndex.MOTOR_DATA_SIZE*i] = (uint8_t)ID.at(i);
    //         //keep the position of data corresponding to the motor id
    //         otherTransferToByte(data[i], &output.buffer[ControlIndex.MOTOR_DATA + ControlIndex.MOTOR_DATA_SIZE*i]);
    //         output.length += ControlIndex.MOTOR_DATA_SIZE;
    //     }
    //     generateCRC(&output.buffer[ControlIndex.FUNCTION_CODE], output.length - 1);
    //     output.length += CRC_DATA_SIZE;
    // }
    
}

#endif
