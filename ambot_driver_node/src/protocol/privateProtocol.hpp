/*  A class of private protocol between PC and MCU
    *
    * Author： chen chen
    * Date: 2024-8-22
    * Email: 1240563221@qq.com
    *
*/

#ifndef __PRIVATE_PROTOCOL_HPP__
#define __PRIVATE_PROTOCOL_HPP__

#include <iostream>
#include <vector>
#include <string.h>
#include "protocolStruct.hpp"
#include "crcModbus.hpp"

namespace ambot_driver_ns
{

    class PrivateProtocolCLASS
    {
    private:
        MotorLimitation_TP motorLimit;

    public:
        MotorRunMode_TP motorMode;
        FunctionCode_TP FunctionCode;
        ControlIndex_TP ControlIndex;
        ResponseIndex_TP ResponseIndex;
        ControlCommandCode_TP ControlCommandCode;
        RequestCommandCode_TP RequestCommandCode;
        SensorDataIndex_TP sensorDataIndex;
        MotorOffLineExceptionIndex_TP motorExceptionIndex;

        PrivateProtocolCLASS(/* args */);
        ~PrivateProtocolCLASS();

        bool checkProtocolFrame(const uint8_t *buff, const uint8_t function, const uint8_t command);
        bool crcAuthentication(uint8_t *buff, const uint8_t size);

        bool checkMotorFBFrame(const uint8_t type, const uint8_t *buff);
        bool checkSensorFBFrame(const uint8_t *buff);
        void allMotorStateAnalysis(uint8_t* buffer, MotorFB_TP &output);
        bool motorFBParas(uint8_t *frameBuff, uint16_t size, std::vector<MotorFB_TP> &output);
        bool sensorDataParas(uint8_t *frameBuff, uint16_t size, SensorFB_TP &output);
        bool requestIDParas(uint8_t *frameBuff, uint16_t size, std::vector<int> &id);


        void createGetIDFrame(protocolOutputBuffer_TP &buffer);
        void createCommandFrame(const uint8_t functionCode, const uint8_t commandNum, const protocolInputBuffer_TP& in, protocolOutputBuffer_TP &output);
        template<typename T> void createCommandFrame(const uint8_t functionCode, const uint8_t commandNum, const std::vector<int> &ID, const std::vector<T>& data, protocolOutputBuffer_TP &output);
        template<typename T> void createSetAllMotorOneFeature(const uint8_t inputCommandCode, const std::vector<T> &inputData, protocolOutputBuffer_TP &output);
        template<typename T> void createGetAllMotorOneFeedback(const uint8_t inputCommandCode, const std::vector<T>& inputData, protocolOutputBuffer_TP &output);
        template<typename T> void otherTransferToByte(const T value, uint8_t* buffer);
        template<typename T> void byteTransferToOther(T& goalValue, const uint8_t* buffer);
        void uint16ToPoint(uint16_t input, uint8_t* des);
        float uint16ToFloat(uint16_t input, float min, float max);
        void floatToUint16(const float input, float min, float max, uint8_t* des);
    };

    /**  
    *   @brief      the data transfer function: float to byte
        Parameters:
    *   @param      value    	[in]the reference of goal value 
    *   @param      buffer 		[in]the pointer of wait convert 
    *   @return     none
        */
    template<typename T> void PrivateProtocolCLASS::otherTransferToByte(const T value, uint8_t* buffer)
    {
        T temp = value;
        uint8_t* point = (uint8_t*)&temp;
        for (int i = 0; i < sizeof(T); i++)
            buffer[i] = *(point+i);
    }

    /**  
    *   @brief      the data transfer function: float to byte
        Parameters:
    *   @param      goalValue    	[in]the reference of goal value 
    *   @param      buffer 		    [in]the pointer of wait convert 
    *   @return     none
        */
    template<typename T> void PrivateProtocolCLASS::byteTransferToOther(T& goalValue, const uint8_t* buffer)
    {
        T temp;
        uint8_t* point = (uint8_t*)&temp;
        for (int i = 0; i < sizeof(T); i++)
            *(point + i) = *(buffer + i);
        goalValue = temp;
    }

    /**  
    *   @brief      create command data frame for control command
        Parameters:
    *   @param      functionCode    [in]the function code
    *   @param      commandNum    	[in]the command code
    *   @param      data 			[in]the input data
    *   @param      output 			[out]the data wait to build/create 
    *   @return     none
        */
    template<typename T>
    void PrivateProtocolCLASS::createCommandFrame(const uint8_t functionCode, const uint8_t commandNum, const std::vector<int> &ID, const std::vector<T>& data, protocolOutputBuffer_TP &output)
    {
        uint8_t number =  data.size();
        memset(output.buffer, 0, WRITE_BUFFER_SIZE);

        output.buffer[ControlIndex.FRAME_HEAD] = FRAME_HEAD_DATA;
        output.buffer[ControlIndex.FUNCTION_CODE] = functionCode;
        output.buffer[ControlIndex.COMMAND_CODE] = commandNum;
        output.buffer[ControlIndex.MOTOR_NUM] = number;
        output.length = ControlIndex.DATA_LENGTH;
        for (int i = 0; i < number; i++)
        {
            output.buffer[ControlIndex.MOTOR_ID + ControlIndex.MOTOR_DATA_SIZE*i] = (uint8_t)ID.at(i);
            //keep the position of data corresponding to the motor id
            otherTransferToByte(data[i], &output.buffer[ControlIndex.MOTOR_DATA + ControlIndex.MOTOR_DATA_SIZE*i]);
            output.length += ControlIndex.MOTOR_DATA_SIZE;
        }
        generateCRC(&output.buffer[ControlIndex.FUNCTION_CODE], output.length - 1);
        output.length += CRC_DATA_SIZE;
    }
    
}

#endif
