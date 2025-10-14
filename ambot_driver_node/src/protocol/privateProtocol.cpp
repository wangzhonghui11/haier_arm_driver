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
    PrivateProtocolCLASS::PrivateProtocolCLASS(/* args */)
    {
        motorLimit = {4*MY_PI, -4*MY_PI, 30, -30, 12, -12, 500., 0, 5., 0};
    }
    
    PrivateProtocolCLASS::~PrivateProtocolCLASS()
    {
    }
    
    /**  
    *   @brief      create get ID protocol frame
        Parameters:
    *   @param      output  [out]the frame buffer wait to send
    *   @return     none
        */
    void PrivateProtocolCLASS::createGetIDFrame(protocolOutputBuffer_TP &output)
    {
        memset(output.buffer, 0, WRITE_BUFFER_SIZE);
        output.buffer[ControlIndex.FRAME_HEAD] = FRAME_HEAD_DATA;
        output.buffer[ControlIndex.FUNCTION_CODE] = FunctionCode.REQUEST;
        output.buffer[ControlIndex.COMMAND_CODE] = RequestCommandCode.GET_ID;
        output.length = ControlIndex.DATA_LENGTH - 1;
        generateCRC(&output.buffer[ControlIndex.FUNCTION_CODE], output.length - 1); // do not need frame head
        output.length += CRC_DATA_SIZE;
    }

    /**  
    *   @brief      analysis receive uart data and update motor state
        Parameters:
    *   @param      buff        [in]a pointer for wait to check it whether if control response frame
    *   @param      function    [in]check function code
    *   @param      command     [in]check command code
    *   @return     bool        true:protocol frame      false:not protocol frame
        */
    bool PrivateProtocolCLASS::checkProtocolFrame(const uint8_t *buff, const uint8_t function, const uint8_t command)
    {
        if (*(buff + ControlIndex.FRAME_HEAD) == FRAME_HEAD_DATA && 
            *(buff + ControlIndex.FUNCTION_CODE) == function && 
            *(buff + ControlIndex.COMMAND_CODE) == command)
            return true;
        else if (*(buff + ControlIndex.FRAME_HEAD) == FRAME_HEAD_DATA && 
            *(buff + ControlIndex.FUNCTION_CODE) == function && 
            *(buff + ControlIndex.COMMAND_CODE) == ControlCommandCode.DISABLE_MOTOR)
        {
            printf("receive disable command feedback\n");
            return false;
        }
        else
            return false;
    }

    /**  
    *   @brief      crc authentication: check the crc whether if right
        Parameters:
    *   @param      buff        [in]a pointer for wait to check it whether if control response frame
    *   @param      size        [in]data length
    *   @return     bool        true:pass      false:failed
        */
    bool PrivateProtocolCLASS::crcAuthentication(uint8_t *buff, const uint8_t size)
    {
        static uint8_t crcAuthentication[CRC_DATA_SIZE];
        memcpy(crcAuthentication, &buff[size], CRC_DATA_SIZE);
        generateCRC(buff, size);
        if (!memcmp(&buff[size], crcAuthentication, CRC_DATA_SIZE))
            return true;
        else 
            return false;
    }

    /**  
    *   @brief      analysis receive uart data and update motor state
        Parameters:
    *   @param      type      [in]robot type; 0 - ambot_N1; 1 - ambot_W1; 2 - ambot_P1
    *   @param      buff      [in]a pointer for wait to check it whether if control response frame
    *   @return     bool        true:protocol frame      false:not protocol frame
        */
    bool PrivateProtocolCLASS::checkMotorFBFrame(const uint8_t type, const uint8_t *buff)
    {
        if (type == 0)
        {
            if (*(buff + ControlIndex.FRAME_HEAD) == FRAME_HEAD_DATA && 
            *(buff + ControlIndex.FUNCTION_CODE) == FunctionCode.REQUEST_RESPOND)
                return true;
            else
                return false;
        }else 
        {
            if (*(buff + ControlIndex.FRAME_HEAD) == FRAME_HEAD_DATA && 
            *(buff + ControlIndex.FUNCTION_CODE) == FunctionCode.CONTROL_RESPOND && 
            *(buff + ControlIndex.COMMAND_CODE) == ControlCommandCode.SET_LOCOMOTION_CONTROL)
                return true;
            else
                return false;
        }
    }

    /**  
    *   @brief      analysis receive uart data and update motor state
        Parameters:
    *   @param      buff      [in]a pointer for wait to check it whether if control response frame
    *   @return     bool        true:protocol frame      false:not protocol frame
        */
    bool PrivateProtocolCLASS::checkSensorFBFrame(const  uint8_t *buff)
    {
        if (*(buff + ControlIndex.FRAME_HEAD) == FRAME_HEAD_DATA && 
            *(buff + ControlIndex.FUNCTION_CODE) == FunctionCode.REQUEST_RESPOND && 
            *(buff + ControlIndex.COMMAND_CODE) == RequestCommandCode.GET_SENSOR_IMU)
            return true;
        else
            return false;
    }

    /**  
    *   @brief      analysis receive uart data and update motor state
        Parameters:
    *   @param      buffer      [in]a pointer for receive uart data,wait analysis 
    *   @param      output      [out]a reference of all motor feedback state, wait update 
    *   @return     none
        */
    void PrivateProtocolCLASS::allMotorStateAnalysis(uint8_t* buffer, MotorFB_TP &output)
    {
        uint16_t temp = 0;
        output.id = *(buffer + 0);
        temp = (*(buffer + 1) << 8) | (*(buffer + 2)) ; 
        output.pos = uint16ToFloat(temp, motorLimit.minPosition, motorLimit.maxPosition);   //range:-4pi~4pi
        temp = (*(buffer + 3) << 8) | (*(buffer + 4)); 
        output.vel = uint16ToFloat(temp, motorLimit.minVelocity, motorLimit.maxVelocity);   //range:-30~30 rad/s
        temp = (*(buffer + 5) << 8) | (*(buffer + 6)); 
        output.tor = uint16ToFloat(temp, motorLimit.minCurrent, motorLimit.maxCurrent);     //range:-12~12 N*m
        temp = (*(buffer + 7) << 8) | (*(buffer + 8)); 
        output.state = temp;                                           // exception state flag
    }

    /**  
    *   @brief      a app interface to paras motor feedback frame
        Parameters:
    *   @param      frameBuff       [in]the pointer of buff wait to paras
    *   @param      size    	    [in]buffer size
    *   @param      output 			[out]all motor feedback data 
    *   @return     none
        */
    bool PrivateProtocolCLASS::motorFBParas(uint8_t *frameBuff, uint16_t size, std::vector<MotorFB_TP> &output)
    {
        static uint8_t resMotorNum, frameDataLength;
        static uint8_t crcAuthentication[CRC_DATA_SIZE];
        static const uint8_t oneMotorDataLength = 9;

        resMotorNum = frameBuff[ResponseIndex.MOTOR_NUM];
        frameDataLength = resMotorNum*oneMotorDataLength + CRC_DATA_SIZE + ResponseIndex.DATA_LENGTH;
        memcpy(crcAuthentication, &frameBuff[frameDataLength - CRC_DATA_SIZE], CRC_DATA_SIZE);
        generateCRC(frameBuff, frameDataLength - CRC_DATA_SIZE);
        if (output.size() >= resMotorNum)
        {
            if (!memcmp(&frameBuff[frameDataLength  - CRC_DATA_SIZE], crcAuthentication, CRC_DATA_SIZE))
            {
                for (int i = 0; i < resMotorNum; i++)
                {   
                    // printf("i:%d   id:%d\n", i, frameBuff[ResponseIndex.MOTOR_ID + oneMotorDataLength*i]);
                    allMotorStateAnalysis(&frameBuff[ResponseIndex.MOTOR_ID + oneMotorDataLength*i], output.at(i));
                    if (output.at(i).id > 16)
                    {
                        std::cout << COUT_RED << "bug here! print all raw data!" << COUT_RESET <<std::endl;
                        for (int j = 0; j < size; j++)
                        {
                            printf("%x \t", frameBuff[j]);
                        }
                        printf("\n");
                        // exit(0);
                    }
                    
                }
                resMotorNum = 0; 
                memset(crcAuthentication, 0 , CRC_DATA_SIZE);
                return true;
            }else
            {
                return false;
            }
        }
        else
        {
            // printf("num failed\n");
            return false;
        }
        
    }

    /**  
    *   @brief      a app interface to paras motor feedback frame
        Parameters:
    *   @param      frameBuff       [in]the pointer of buff wait to paras
    *   @param      size    	    [in]buffer size
    *   @param      output 			[out]sensor data 
    *   @return     none
        */
    bool PrivateProtocolCLASS::sensorDataParas(uint8_t *frameBuff, uint16_t size, SensorFB_TP &output)
    {
        static const uint16_t frameDataLength = sensorDataIndex.ALL_DATA_NUM;
        static uint8_t crcAuthentication[CRC_DATA_SIZE];

        // memcpy(sensorReadBuffer, &tempReadBuffer[frameHeadIndex + 1], frameDataLength - 1);             //-1:dont need the frame head
        memcpy(crcAuthentication, &frameBuff[sensorDataIndex.CRC_AUTHENTICATION], CRC_DATA_SIZE);
        generateCRC(frameBuff, sensorDataIndex.CRC_AUTHENTICATION);
        if (!memcmp(&frameBuff[sensorDataIndex.CRC_AUTHENTICATION], crcAuthentication, CRC_DATA_SIZE))
        {
            #ifdef SHOW_READ_FEEDBACK_PERIOD
            gettimeofday(&timeNewSensor, NULL);
            printf("sensor feedback period time:%ld\n", (timeNewSensor.tv_sec*1000000 + timeNewSensor.tv_usec) - (timeOldSensor.tv_sec*1000000 + timeOldSensor.tv_usec));
            timeOldSensor = timeNewSensor;
            #endif
            byteTransferToOther(output.acceleration.x, &frameBuff[sensorDataIndex.IMU_LINE_ACC_X]);
            byteTransferToOther(output.acceleration.y, &frameBuff[sensorDataIndex.IMU_LINE_ACC_Y]);
            byteTransferToOther(output.acceleration.z, &frameBuff[sensorDataIndex.IMU_LINE_ACC_Z]);
            byteTransferToOther(output.gyroscope.x, &frameBuff[sensorDataIndex.IMU_ANGULAR_X]);
            byteTransferToOther(output.gyroscope.y, &frameBuff[sensorDataIndex.IMU_ANGULAR_Y]);
            byteTransferToOther(output.gyroscope.z, &frameBuff[sensorDataIndex.IMU_ANGULAR_Z]);
            byteTransferToOther(output.quaternion.w, &frameBuff[sensorDataIndex.IMU_QUATERNION_W]);
            byteTransferToOther(output.quaternion.x, &frameBuff[sensorDataIndex.IMU_QUATERNION_X]);
            byteTransferToOther(output.quaternion.y, &frameBuff[sensorDataIndex.IMU_QUATERNION_Y]);
            byteTransferToOther(output.quaternion.z, &frameBuff[sensorDataIndex.IMU_QUATERNION_Z]);
            // ambotState.force.touch.FL = frameBuff[sensorDataIndex.TOUCH_FL];
            // ambotState.force.touch.FR = frameBuff[sensorDataIndex.TOUCH_FR];
            // ambotState.force.touch.HL = frameBuff[sensorDataIndex.TOUCH_HL];
            // ambotState.force.touch.HR = frameBuff[sensorDataIndex.TOUCH_HR];
            // ambotState.force.shift.FL = frameBuff[sensorDataIndex.SHIFT_FL];
            // ambotState.force.shift.FR = frameBuff[sensorDataIndex.SHIFT_FR];
            // ambotState.force.shift.HL = frameBuff[sensorDataIndex.SHIFT_HL];
            // ambotState.force.shift.HR = frameBuff[sensorDataIndex.SHIFT_HR];
            memset(crcAuthentication, 0 , CRC_DATA_SIZE);
            return true;
        }else
        {
            return false;
        }
    }

    /**  
    *   @brief      a app interface to paras motor ID
        Parameters:
    *   @param      frameBuff       [in]the pointer of buff wait to paras
    *   @param      size    	    [in]buffer size
    *   @param      id 			[out]sensor data 
    *   @return     none
        */
    bool PrivateProtocolCLASS::requestIDParas(uint8_t *frameBuff, uint16_t size, std::vector<int> &id)
    {
        static uint8_t crcAuthentication[CRC_DATA_SIZE];
        uint16_t frameDataLength = size - CRC_DATA_SIZE;
        uint8_t motorNum = 0;

        // memcpy(sensorReadBuffer, &tempReadBuffer[frameHeadIndex + 1], frameDataLength - 1);             //-1:dont need the frame head
        memcpy(crcAuthentication, &frameBuff[frameDataLength], CRC_DATA_SIZE);
        generateCRC(frameBuff, frameDataLength);
        if (!memcmp(&frameBuff[frameDataLength], crcAuthentication, CRC_DATA_SIZE))
        {
            #ifdef SHOW_READ_FEEDBACK_PERIOD
            gettimeofday(&timeNewSensor, NULL);
            printf("sensor feedback period time:%ld\n", (timeNewSensor.tv_sec*1000000 + timeNewSensor.tv_usec) - (timeOldSensor.tv_sec*1000000 + timeOldSensor.tv_usec));
            timeOldSensor = timeNewSensor;
            #endif

            motorNum = frameBuff[ResponseIndex.MOTOR_NUM];
            id.resize(motorNum);

            for (int i = 0; i < motorNum; i++)
            {
                id.at(i) = frameBuff[ResponseIndex.MOTOR_ID + ResponseIndex.MOTOR_DATA_SIZE*i];
                printf("ID%d : %d\n", i, id.at(i));
            }
            memset(crcAuthentication, 0 , CRC_DATA_SIZE);
            return true;
        }else
            return false;
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
    void PrivateProtocolCLASS::createCommandFrame(const uint8_t functionCode, const uint8_t commandNum, const protocolInputBuffer_TP& in, protocolOutputBuffer_TP &output)
    {
        constexpr uint8_t posIndex = 0, velIndex = 2, curIndex = 4, KpIndex = 6, KdIndex = 8;
        uint8_t number =  in.number;
        memset(output.buffer, 0, WRITE_BUFFER_SIZE);

        output.buffer[ControlIndex.FRAME_HEAD] = FRAME_HEAD_DATA;
        output.buffer[ControlIndex.FUNCTION_CODE] = functionCode;
        output.buffer[ControlIndex.COMMAND_CODE] = commandNum;
        output.buffer[ControlIndex.MOTOR_NUM] = number;
        output.length = ControlIndex.DATA_LENGTH;
        for (int i = 0; i < number; i++)
        {
            if (in.command.at(i).id <= 0 || in.command.at(i).id > 16)
            {
                std::cout << "write command id wrong, index:" << i << ", ID:" << in.command.at(i).id << std::endl;
            }
            // printf("cmd  id:%d\n", in.command.at(i).id);
            output.buffer[ControlIndex.MOTOR_ID + ControlIndex.LOCOMOTION_CONTROL_DATA_SIZE*i] = in.command.at(i).id;
            floatToUint16(in.command.at(i).q, motorLimit.minPosition, motorLimit.maxPosition,  &output.buffer[ControlIndex.MOTOR_DATA + ControlIndex.LOCOMOTION_CONTROL_DATA_SIZE*i + posIndex]);
            floatToUint16(in.command.at(i).dq, motorLimit.minVelocity, motorLimit.maxVelocity,  &output.buffer[ControlIndex.MOTOR_DATA + ControlIndex.LOCOMOTION_CONTROL_DATA_SIZE*i + velIndex]);
            floatToUint16(in.command.at(i).tor, motorLimit.minCurrent, motorLimit.maxCurrent,  &output.buffer[ControlIndex.MOTOR_DATA + ControlIndex.LOCOMOTION_CONTROL_DATA_SIZE*i + curIndex]);
            floatToUint16(in.command.at(i).kp, motorLimit.minKp, motorLimit.maxKp,  &output.buffer[ControlIndex.MOTOR_DATA + ControlIndex.LOCOMOTION_CONTROL_DATA_SIZE*i + KpIndex]);
            floatToUint16(in.command.at(i).kd, motorLimit.minKd, motorLimit.maxKd,  &output.buffer[ControlIndex.MOTOR_DATA + ControlIndex.LOCOMOTION_CONTROL_DATA_SIZE*i + KdIndex]);
            // floatToUint16(in.command.at(in.command.at(i).id - 1).kd, motorLimit.minKd, motorLimit.maxKd,  &output.buffer[ControlIndex.MOTOR_DATA + ControlIndex.LOCOMOTION_CONTROL_DATA_SIZE*i + KdIndex]);
            output.length += ControlIndex.LOCOMOTION_CONTROL_DATA_SIZE;
            
        }
        generateCRC(&output.buffer[ControlIndex.FUNCTION_CODE], output.length - 1);
        output.length += CRC_DATA_SIZE;
    }

    /**  
    *   @brief      motor set,include: motor mode, max angle. min angle, motor velocity
        Parameters:
    *   @param      inputCommandCode    [in]input command code
    *   @param      inputData           [in]set data
    *   @param      output              [out]the frame created
    *   @return     true:get successful   false:get failed
        */
    template<typename T> void PrivateProtocolCLASS::createSetAllMotorOneFeature(const uint8_t inputCommandCode, const std::vector<T> &inputData, protocolOutputBuffer_TP &output)
    {
        createCommandFrame(FunctionCode.CONTROL, inputCommandCode, inputData, output);
    }

    /**  
    *   @brief      create request frame, the read params include: motor mode, max angle. min angle, motor velocity
        Parameters:
    *   @param      inputCommandCode    [in]input command code
    *   @param      inputData           [in]set data
    *   @param      output              [out]the frame created
    *   @return     true:get successful   false:get failed
        */
    template<typename T> void PrivateProtocolCLASS::createGetAllMotorOneFeedback(const uint8_t inputCommandCode, const std::vector<T> &inputData, protocolOutputBuffer_TP &output)
    {
        createCommandFrame(FunctionCode.REQUEST, inputCommandCode, inputData, output);
    }


    /**  
    *   @brief      the data transfer function: uint16 to float and return data
        Parameters:
    *   @param      input    	[in]input data, wait to transfer
    *   @param      min 	    [in]the min limit of transfer data 
    *   @param      max    	    [in]the max limit of transfer data 
    *   @return     the transfer data
        */
    float PrivateProtocolCLASS::uint16ToFloat(uint16_t input, float min, float max)
    {
        return (float)((max - min) * input / UINT16_MAX) + min;
    }

    /**  
    *   @brief      the data transfer function: float to uint16 and save to pointer
        Parameters:
    *   @param      input    	[in]input data, wait to transfer
    *   @param      min 	    [in]the min limit of transfer data 
    *   @param      max    	    [in]the max limit of transfer data 
    *   @param      des 	    [in]the destination pointer of save addr
    *   @return     none
        */
    void PrivateProtocolCLASS::floatToUint16(const float input, float min, float max, uint8_t* des)
    {
        uint16_t result;
        if (input > max)
            result = UINT16_MAX;
        else if (input < min)
            result = 0;
        else
            result = (uint16_t)(UINT16_MAX * (input - min)/(max - min));
        uint16ToPoint(result, des);
    }

    void PrivateProtocolCLASS::uint16ToPoint(uint16_t input, uint8_t* des)
    {
        *des = input >> 8;
        *(des + 1) = input;
    }

    
}