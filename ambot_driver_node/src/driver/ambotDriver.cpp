/*  A class include yangtze river inspection robot hardware driver
    *
    * Author： 
    * Date: 2024-8-22
    * Email:
    *
*/

#include "ambotDriver.hpp"

namespace ambot_driver_ns{

    AmbotDriverCLASS::AmbotDriverCLASS(RobotDriver_TP &input) : robotParams(input)
    {
        // protocol = new PrivateProtocolCLASS();


        robotType.push_back("ambot_N1");
        robotType.push_back("ambot_W1");
        robotType.push_back("ambot_P1");
        // 1.open device file
        motorFd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY );
        if (motorFd == -1) {
            perror("Error opening motor serial port");
            exit(0);
        }
        // 2.init serial config file
        struct termios SerialPortSettings;
        struct serial_struct ser_info;
        // 3.1 get serial current feature and save to SerialPortSettings
        tcgetattr(motorFd, &SerialPortSettings);
        // 3.2 according baud to set the serial baudrate
        if (robotParams.motorDevBaud == 115200)
        {
            std::cout  << "motor set baud:" << 115200  << std::endl;  
            cfsetispeed(&SerialPortSettings, B115200);
            cfsetospeed(&SerialPortSettings, B115200);
        }else if (robotParams.motorDevBaud == 921600)
        {
            std::cout  << "motor set baud:" << 921600  << std::endl;
            cfsetispeed(&SerialPortSettings, B921600);
            cfsetospeed(&SerialPortSettings, B921600);
        }else if (robotParams.motorDevBaud == 1000000)
        {
            std::cout << "motor set baud:" << 1000000  << std::endl;
            cfsetispeed(&SerialPortSettings, B1000000);
            cfsetospeed(&SerialPortSettings, B1000000);
        }else if (robotParams.motorDevBaud == 1500000)
        {
            std::cout  << "motor set baud:" << 1500000<< std::endl;
            cfsetispeed(&SerialPortSettings, B1500000);
            cfsetospeed(&SerialPortSettings, B1500000);
        }else if (robotParams.motorDevBaud == 2000000)
        {
            std::cout  << "motor set baud:" << 2000000 << std::endl;
            cfsetispeed(&SerialPortSettings, B2000000);
            cfsetospeed(&SerialPortSettings, B2000000);
        }else if (robotParams.motorDevBaud == 2500000)
        {
            std::cout  << "motor set baud:" << 2500000 << std::endl;
            cfsetispeed(&SerialPortSettings, B2500000);
            cfsetospeed(&SerialPortSettings, B2500000);
        }
        else{
            std::cout << "motor set baud:" << 1000000  << std::endl;
            cfsetispeed(&SerialPortSettings, B1000000);
            cfsetospeed(&SerialPortSettings, B1000000);
        }
        // 3.3 set uart communicate features
        SerialPortSettings.c_cflag &= ~PARENB;      // Disable parity
        SerialPortSettings.c_cflag &= ~CSTOPB;      // 1 stop bit
        SerialPortSettings.c_cflag |= CS8;          // 8 bits per byte
        SerialPortSettings.c_cflag &= ~CRTSCTS;     // Disable RTS/CTS hardware flow control
        SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = SerialPortSettings
        SerialPortSettings.c_lflag &= ~ICANON;      // Disable canonical mode
        SerialPortSettings.c_lflag &= ~ECHO;        // Disable echo
        SerialPortSettings.c_lflag &= ~ECHOE;       // Disable erasure
        SerialPortSettings.c_lflag &= ~ECHONL;      // Disable new-line echo
        SerialPortSettings.c_lflag &= ~ISIG;        // Disable interpretation of INTR, QUIT and SUSP
        SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        SerialPortSettings.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
        SerialPortSettings.c_oflag &= ~OPOST;       // Prevent special interpretation of output bytes (e.g. newline chars)
        SerialPortSettings.c_oflag &= ~ONLCR;       // Prevent conversion of newline to carriage return/line feed
        SerialPortSettings.c_cc[VTIME] = 0;         // timeout set, unit 1/10s, if set zero, will return right now 
        SerialPortSettings.c_cc[VMIN] = 0;          // wait for enough data to read, if set zero, data will read right now
        // 3.4 Enable linux FTDI low latency mode
        ioctl(motorFd, TIOCGSERIAL, &ser_info);
        ser_info.flags |= ASYNC_LOW_LATENCY;
        ioctl(motorFd, TIOCSSERIAL, &ser_info);
        // 3.5 Set the new attributes to the termios structure
        if((tcsetattr(motorFd, TCSANOW, &SerialPortSettings)) != 0) 
        {
            std::cout << "ERROR in setting serial port attributes! for port " << robotParams.motorDevName.c_str() << std::endl;
            sleep(1);
        }
        else
            std::cout << "Port " << robotParams.motorDevName.c_str() << " open successfully!" << std::endl;
    }
    
    AmbotDriverCLASS::~AmbotDriverCLASS()
    {
    }
    

    /**  
    *   @brief      send the tty data to mcu
        Parameters:
    *   @param      void
    *   @return     the num of send data
        */
    // ssize_t AmbotDriverCLASS::txPacket(protocolOutputBuffer_TP &out)
    // {
    //     return  write(motorFd, out.buffer, out.length);
    // }


    // /**  
    // *   @brief      wait the response data and analyses data
    //     Parameters:
    // *   @param      commandNumber		[in]the command code 
    // *   @return     true:get successful   false:get failed
    //     */
    // bool AmbotDriverCLASS::waitCommandResponse(const uint8_t commandNumber)
    // {
    //     // 1.init all local variables
    //     struct timeval time_old,time_new;
    //     uint8_t tempReadBuffer[READ_BUFFER_SIZE];
    //     uint8_t currenBuffer[READ_BUFFER_SIZE];
    //     uint8_t currentReadCount, alreadyReadCount, frameHeadIndex;
    //     bool frameHeadFlag = false;
    //     frameHeadIndex = 0;
    //     alreadyReadCount = 0;
    //     // 2.get current time for read timeout
    //     gettimeofday(&time_new, NULL);
    //     time_old = time_new;
    //     // 3.wait for receive data
    //     while (((time_new.tv_sec*U_SECOND_COUNT + time_new.tv_usec)-(time_old.tv_sec*U_SECOND_COUNT + time_old.tv_usec)) < DELAY_TIMEOUT )
    //     {
    //         usleep(WAIT_RESPONSE_US_DELAY);
    //         currentReadCount = read(motorFd, &tempReadBuffer[alreadyReadCount], READ_BUFFER_SIZE - alreadyReadCount);
    //         alreadyReadCount += currentReadCount;
    //         // 4.judge current receive data length whether if enough to analysis
    //         if (alreadyReadCount - frameHeadIndex >= 5)
    //         {
    //             // 5.locate frame head index
    //             for (int i = frameHeadIndex; i < alreadyReadCount; i++)
    //             {
    //                 if (protocol->checkProtocolFrame(&tempReadBuffer[i], protocol->FunctionCode.CONTROL_RESPOND, commandNumber))
    //                 {
    //                     frameHeadIndex = i;
    //                     frameHeadFlag = true;
    //                     break;
    //                 }
    //                 frameHeadFlag = false;
    //             }
    //             // 6.if frame head locate successful, ready to analysis
    //             if (frameHeadFlag)
    //             {
    //                 memcpy(currenBuffer, &tempReadBuffer[frameHeadIndex + protocol->ControlIndex.FUNCTION_CODE], 4);
    //                 if (protocol->crcAuthentication(currenBuffer, 2))
    //                     return true;
    //                 frameHeadIndex++;
    //             }
    //         }
    //         gettimeofday(&time_new, NULL);
    //     }
    //     printf("timeout\n");
    //     return false;
    // }


    // /**  
    // *   @brief      wait the request response data and analyses data
    //     Parameters:
    // *   @param      commandNumber		[in]the command code 
    // *   @return     true:get successful   false:get failed
    //     */
    // bool AmbotDriverCLASS::waitRequestIDResponse(const uint8_t commandNumber, std::vector<int> &id)
    // {
    //     // 1.init all local variables
    //     struct timeval time_old,time_new;
    //     uint8_t tempReadBuffer[READ_BUFFER_SIZE];
    //     uint8_t currenBuffer[READ_BUFFER_SIZE];
    //     uint8_t resMotorNum, currentReadCount, alreadyReadCount;
    //     bool frameHeadFlag = false;
    //     uint8_t frameHeadIndex = 0;
    //     alreadyReadCount = 0;
    //     // 2.get current time for read timeout
    //     gettimeofday(&time_new, NULL);
    //     time_old = time_new;
    //     // 3.wait for receive data
    //     while (((time_new.tv_sec*U_SECOND_COUNT + time_new.tv_usec) - (time_old.tv_sec*U_SECOND_COUNT + time_old.tv_usec)) < DELAY_TIMEOUT )
    //     {
    //         usleep(20*WAIT_RESPONSE_US_DELAY);
    //         currentReadCount = read(motorFd, &tempReadBuffer[alreadyReadCount], READ_BUFFER_SIZE - alreadyReadCount);
    //         // for (int i = 0; i < currentReadCount; i++)
    //         // {
    //         //     printf("%x\t", tempReadBuffer[alreadyReadCount + i]);
    //         // }
    //         alreadyReadCount += currentReadCount;

    //         // 4.make sure get ID response frame and set the frame index to real buff index
    //         for (int i = frameHeadIndex; i < alreadyReadCount; i++)
    //         {
    //             if (protocol->checkProtocolFrame(&tempReadBuffer[i], protocol->FunctionCode.REQUEST_RESPOND, commandNumber))
    //             {
    //                 frameHeadIndex = i;
    //                 frameHeadFlag = true;
    //                 break;
    //             }
    //             frameHeadFlag = false;
    //         }
    //         // 5. ready to analysis ID response frame
    //         if (frameHeadFlag)
    //         {
    //             resMotorNum = tempReadBuffer[frameHeadIndex + protocol->ControlIndex.MOTOR_NUM];
    //             uint8_t dataNum = resMotorNum * protocol->ResponseIndex.MOTOR_DATA_SIZE + 3 + CRC_DATA_SIZE;
    //             // 6.judge current receive data length whether if enough to analysis
    //             if (alreadyReadCount - frameHeadIndex >= dataNum + 1)
    //             {
    //                 memcpy(currenBuffer, &tempReadBuffer[frameHeadIndex + protocol->ControlIndex.FUNCTION_CODE], dataNum);
    //                 if (protocol->requestIDParas(currenBuffer, dataNum, id))
    //                     return true;
    //                 else
    //                     frameHeadIndex++;
    //             }
    //             frameHeadFlag = false;
    //         }
    //         gettimeofday(&time_new, NULL);
    //     }
    //     return false;
    // }

    // /**  
    // *   @brief      motor set,include: motor mode, max angle. min angle, motor velocity
    //     Parameters:
    // *   @param      inputCommandCode    [in]input command code
    // *   @param      inputData           [in]set data
    // *   @return     true:get successful   false:get failed
    //     */
    // template<typename T> bool AmbotDriverCLASS::setAllMotorOneFeature(const uint8_t inputCommandCode, const std::vector<int> &IDBuffer, const std::vector<T> &inputData)
    // {
    //     protocolOutputBuffer_TP sendBuffer;
    //     protocol->createCommandFrame(protocol->FunctionCode.CONTROL, inputCommandCode, IDBuffer, inputData, sendBuffer);
    //     if(sendBuffer.length != txPacket(sendBuffer))
    //         return false;
    //     if(!waitCommandResponse(inputCommandCode))
    //     {
    //         if (inputCommandCode == protocol->ControlCommandCode.SET_ZERO)
    //             std::cout << COUT_RED << "set motor machine zero Failed!!!" <<  COUT_RESET << std::endl;
    //         else if (inputCommandCode == protocol->ControlCommandCode.ENABLE_MOTOR)
    //             std::cout << COUT_RED << "enable all motor Failed!!!" <<  COUT_RESET << std::endl;
    //         else if (inputCommandCode == protocol->ControlCommandCode.DISABLE_MOTOR)
    //             std::cout << COUT_RED << "disable all motor Failed!!!" <<  COUT_RESET << std::endl;
    //         else if (inputCommandCode == protocol->ControlCommandCode.SELECT_MODE)
    //             std::cout << COUT_RED << "set motor mode Failed!!!" <<  COUT_RESET << std::endl;
    //         else if (inputCommandCode == protocol->ControlCommandCode.SET_MAX_ANGLE)
    //             std::cout << COUT_RED << "set max limit angle Failed!!!" <<  COUT_RESET << std::endl;
    //         else if (inputCommandCode == protocol->ControlCommandCode.SET_MIN_ANGLE)
    //             std::cout << COUT_RED << "set min limit angle Failed!!!" <<  COUT_RESET << std::endl;
    //         else if (inputCommandCode == protocol->ControlCommandCode.SET_RUNNING_VELOCITY)
    //             std::cout << COUT_RED << "set motor velocity Failed!!!" <<  COUT_RESET << std::endl;
    //         return false;
    //     }
    //     else
    //     {
    //         if (inputCommandCode == protocol->ControlCommandCode.SET_ZERO)
    //             std::cout << COUT_GREEN << "set motor machine zero OK!!!" <<  COUT_RESET << std::endl;
    //         else if (inputCommandCode == protocol->ControlCommandCode.ENABLE_MOTOR)
    //             std::cout << COUT_GREEN << "enable all motor OK!!!" <<  COUT_RESET << std::endl;
    //         else if (inputCommandCode == protocol->ControlCommandCode.DISABLE_MOTOR)
    //             std::cout << COUT_GREEN << "disable all motor OK!!!" <<  COUT_RESET << std::endl;
    //         else if (inputCommandCode == protocol->ControlCommandCode.SELECT_MODE)
    //             std::cout << COUT_GREEN << "set motor mode OK!!!" <<  COUT_RESET << std::endl;
    //         else if (inputCommandCode == protocol->ControlCommandCode.SET_MAX_ANGLE)
    //             std::cout << COUT_GREEN << "set max limit angle OK!!!" <<  COUT_RESET << std::endl;
    //         else if (inputCommandCode == protocol->ControlCommandCode.SET_MIN_ANGLE)
    //             std::cout << COUT_GREEN << "set min limit angle OK!!!" <<  COUT_RESET << std::endl;
    //         else if (inputCommandCode == protocol->ControlCommandCode.SET_RUNNING_VELOCITY)
    //             std::cout << COUT_GREEN << "set motor velocity OK!!" <<  COUT_RESET << std::endl;
    //         return true;
    //     }
    // }

    // /**  
    // *   @brief      get all motor id and motor num
    //     Parameters:
    // *   @param      none
    // *   @return     true:get successful   false:get failed
    //     */
    // bool AmbotDriverCLASS::requestAllMotorID(std::vector<int> &out)
    // {
    //     protocolOutputBuffer_TP txBuff;
    //     // 1. build git all motor information request frame
    //     protocol->createGetIDFrame(txBuff);

    //     // 2.send request frame
    //     if(txBuff.length != txPacket(txBuff))
    //         return false;
    //     // 3.wait for request response frame
    //     if(waitRequestIDResponse(protocol->RequestCommandCode.GET_ID, out))
    //         return true;
    //     else
    //         return false;
    // }

    /**  
    *   @brief      initial function for MCU class
        Parameters:
    *   @param      void
    *   @return     none
        */
    bool AmbotDriverCLASS::initial(void)
    {
        threadStop = false;
        std::cout<< "\nRobot Type : " << robotParams.robotType  << std::endl;
        printf("ready to init\n\n");
        // if(requestAllMotorID(motorIDExist))
        // {
        //     std::cout << COUT_GREEN << "getID OK!!!\n" <<  COUT_RESET << std::endl;
        //     motorNumExist = motorIDExist.size();
        //     if (motorNumExist != robotParams.motorNum)
        //         std::cout << COUT_RED << "get motor number is not equal with the yaml file configuration\n" <<  COUT_RESET << std::endl;
            
        //     motorCommandExist.number = motorNumExist;
        //     motorCommandExist.command.resize(motorNumExist);
        //     std::vector<uint32_t> intSetupData(motorNumExist, 0);
        //     for (int i = 0; i < motorNumExist; i++)
        //     {
        //         intSetupData.at(i) = robotParams.motorControlMode.at(motorIDExist.at(i) - 1);
        //     }
            
        //     setAllMotorOneFeature(protocol->ControlCommandCode.SELECT_MODE, motorIDExist, intSetupData);
            
        //     if (robotParams.robotType == robotType.at(ambot_W1))
        //         setAllMotorOneFeature(protocol->ControlCommandCode.ENABLE_MOTOR, motorIDExist, intSetupData);
        //     else if (robotParams.robotType == robotType.at(ambot_N1))
        //     {
        //         setAllMotorOneFeature(protocol->ControlCommandCode.ENABLE_MOTOR, motorIDExist, intSetupData);
        //         std::vector<float> in(motorNumExist, 0);
        //         for (int i = 0; i < motorNumExist; i++)
        //             in.at(i) = robotParams.motorMaxAngle.at(motorIDExist.at(i) - 1);
        //         setAllMotorOneFeature(protocol->ControlCommandCode.SET_MAX_ANGLE, motorIDExist, in);
        //         for (int i = 0; i < motorNumExist; i++)
        //             in.at(i) = robotParams.motorMinAngle.at(motorIDExist.at(i) - 1);
        //         setAllMotorOneFeature(protocol->ControlCommandCode.SET_MIN_ANGLE, motorIDExist, in);
        //     }
             createReceiveThread();
            return true;
        // }
        // else
        // {
        //     std::cout << COUT_RED << "getID failed!!!\n" <<  COUT_RESET << std::endl;
        //     return false;
        // }
    }

    /**  
    *   @brief      disable all motor, the interface open to user
        Parameters:
    *   @param      none
    *   @return     true:get successful   false:get failed
        */
    // bool AmbotDriverCLASS::disableAllMotor(void)
    // {
    //     // try three times to stop 
    //     static std::vector<uint32_t> temp(motorNumExist, 0);
    //     for (int i = 0; i < 3; i++)
    //     {
    //         std::cout << COUT_BLUE << "try " << i+1 << " to stop robot"  << COUT_RESET << std::endl;
            
    //         if(setAllMotorOneFeature(protocol->ControlCommandCode.DISABLE_MOTOR, motorIDExist, temp))
    //             break;
    //     }
    //     // setAllMotorOneFeature(ControlCommandCode.DISABLE_MOTOR, (uint32_t)0);
    //     std::cout << "close motor and sensor FD!" << std::endl;
    //     close(motorFd);
    //     close(sensorFd);
    //     return 0;
    // }

    // /**  
    // *   @brief      according raw motor feedback set robot ros message value
    //     Parameters:
    // *   @param      input   [in]raw motor feedback 
    // *   @param      output  [in]the quote of ros message 
    // *   @return     none
    //     */
    // void AmbotDriverCLASS::motorRawState2RosPublish(std::vector<MotorFB_TP> &input, ambot_msg::AmbotState &output)
    // {
    //     output.motor_num = input.size();
    //     for (int i = 0; i < input.size(); i++)
    //     {
    //         if (input.at(i).id > 16 || input.at(i).id < 0)
    //         {
    //             std::cout << "bug here! print all infos! input.size():" << input.size() << std::endl;
    //             for (int j = 0; j < input.size(); j++)
    //             {
    //                 printf("id:%d\t pos:%f vel:%f cur:%f state:%x\n",
    //                 input.at(j).id,
    //                 input.at(j).pos,
    //                 input.at(j).vel,
    //                 input.at(j).tor,
    //                 input.at(j).state);
    //             }
    //             printf("\n");
    //             // exit(0);
    //             return;
    //         }
            
    //         if (input.at(i).id != 0)
    //         {
    //             output.motorState.at(input.at(i).id - 1).pos = input.at(i).pos;
    //             output.motorState.at(input.at(i).id - 1).vel = input.at(i).vel;
    //             output.motorState.at(input.at(i).id - 1).cur = input.at(i).tor;
    //             output.motorState.at(input.at(i).id - 1).vol = input.at(i).state;
    //         }
    //     }
    // }

    // /**  
    // *   @brief      according raw sensor data set robot ros message value
    //     Parameters:
    // *   @param      in   [in]raw sensor data
    // *   @param      out  [in]the quote of ros message 
    // *   @return     none
    //     */
    // void AmbotDriverCLASS::sensorRaw2RosPublish(const SensorFB_TP &in, ambot_msg::AmbotState &out)
    // {
    //     out.imu.acceleration.x = in.acceleration.x;
    //     out.imu.acceleration.y = in.acceleration.y;
    //     out.imu.acceleration.z = in.acceleration.z;
    //     out.imu.gyroscope.x = in.gyroscope.x;
    //     out.imu.gyroscope.y = in.gyroscope.y;
    //     out.imu.gyroscope.z = in.gyroscope.z;
    //     out.imu.quaternion.w = in.quaternion.w;
    //     out.imu.quaternion.x = in.quaternion.x;
    //     out.imu.quaternion.y = in.quaternion.y;
    //     out.imu.quaternion.z = in.quaternion.z;
    // }

    // /**  
    // *   @brief      get all motor state feedback data
    //     Parameters:
    // *   @param      none
    // *   @return     true:get successful   false:get failed
    //     */
     #define TEMP_READ_BUFFER_LENGTH 256*8
     #define LEN_MAX 36
    void AmbotDriverCLASS::getAllMotorStateFromMCU(void)
    {   
        // 1. init all local variables
        uint8_t inFlag;
        uint8_t currentReadCount;
        int16_t frameHeadIndex, currentTail;
        uint8_t tempReadBuffer[TEMP_READ_BUFFER_LENGTH];
        uint8_t motorFeedbackBuffer[TEMP_READ_BUFFER_LENGTH];
        uint8_t read_buffer[36];
        // 2. set init value to variables
        frameHeadIndex = currentTail = 0;

        // 3.wait for receive enough data to analysis
        while (1)
        {
            // 4. if detect ctrl+C, current thread will exit 
            if (threadStop)
            {
                printf("receive data thread exit!!\n");
                pthread_exit(NULL);
            }
            usleep(10000);
            // 6.read data and analysis
            /** ssize_t read(int fd, void *buf, size_t count);
            ​​参数​​:
            fd: 文件描述符（在您代码中是 motorFd）
            buf: 存储读取数据的缓冲区指针（&tempReadBuffer[currentTail]）
            count: 要读取的最大字节数（TEMP_READ_BUFFER_LENGTH - currentTail）
            ​​返回值​​:
            成功时返回实际读取的字节数（currentReadCount）
            返回0表示到达文件末尾（对串口通常意味着连接断开）
            返回-1表示出错，错误代码在 errno中
            */
            currentReadCount = read(motorFd, read_buffer, LEN_MAX);
            std::cout << "Read " << currentReadCount << " bytes:\n";
            for (ssize_t i = 0; i < currentReadCount; ++i) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') 
                        << static_cast<int>(read_buffer[i]) << " ";
}
        }
    }

    // /**  
    // *   @brief      get all sensor data, include imu and force sensor touch 
    //     Parameters:
    // *   @param      none
    // *   @return     true:get successful   false:get failed
    //     */
    // void AmbotDriverCLASS::getSensorDataFromMCU(void)
    // {   
    //     // 1. init all local variables
    //     uint8_t currentReadCount;
    //     uint16_t frameHeadIndex, currentTail;
    //     uint8_t tempReadBuffer[TEMP_READ_BUFFER_LENGTH];
    //     uint8_t sensorReadBuffer[READ_BUFFER_SIZE];

    //     // 2. set init value to variables
    //     frameHeadIndex = currentTail = 0;
    //     const uint16_t frameDataLength = protocol->sensorDataIndex.ALL_DATA_NUM + 1; //add a frameHead
        
    //     #ifdef SHOW_READ_FEEDBACK_PERIOD
    //     struct timeval timeOldSensor,timeNewSensor;
    //     gettimeofday(&timeNewSensor, NULL);
    //     timeOldSensor = timeNewSensor;
    //     #endif

    //     // 3.wait for receive enough data to analysis
    //     while (1)
    //     {
    //         // 4. if detect ctrl+C, current thread will exit 
    //         if (threadStop)
    //         {
    //             printf("sensor data receive thread exit!!\n");
    //             pthread_exit(NULL);
    //         }
    //         usleep(WAIT_RESPONSE_US_DELAY);
    //         // 5.keep current receive buffer won't break, keep circle read buffer normal
    //         if (currentTail + 2 * frameDataLength >= TEMP_READ_BUFFER_LENGTH)
    //         {
    //             if(currentTail < frameHeadIndex)
    //             {
    //                 frameHeadIndex = 0;
    //                 currentTail = 0;
    //             }
    //             else{
    //                 memcpy(tempReadBuffer, &tempReadBuffer[frameHeadIndex], currentTail - frameHeadIndex);
    //                 currentTail -= frameHeadIndex;
    //                 frameHeadIndex = 0;
    //                 memset(&tempReadBuffer[currentTail], 0, TEMP_READ_BUFFER_LENGTH - currentTail);
    //             }
    //         }
    //         // 6.read data and analysis
    //         currentReadCount = read(sensorFd, &tempReadBuffer[currentTail], TEMP_READ_BUFFER_LENGTH - currentTail);
    //         currentTail += currentReadCount;
    //         for (int i = frameHeadIndex; i < currentTail; i++)
    //         {
    //             if (protocol->checkSensorFBFrame(&tempReadBuffer[i]))
    //             {
    //                 frameHeadIndex = i;
    //                 break;
    //             }
    //         }
    //         if (currentTail - frameHeadIndex >= frameDataLength)
    //         {
    //             // do not copy the frame head
    //             memcpy(sensorReadBuffer, &tempReadBuffer[frameHeadIndex + protocol->ControlIndex.FUNCTION_CODE], frameDataLength - 1);             //-1:dont need the frame head
    //             if (protocol->sensorDataParas(sensorReadBuffer, frameDataLength - 1, sensorFB))
    //                 sensorRaw2RosPublish(sensorFB, rosData);
    //             #ifdef AMBOT_DEBUG
    //             else
    //             {
    //                 if (currentReadCount != 0)
    //                     printf("imu receive\n");
    //                 std::cout << COUT_RED << "sensor data analysis failed!" << COUT_RESET << std::endl;
    //                 // for (int i = 0; i < currentTail - frameHeadIndex; i++)
    //                 // {   
    //                 //     printf("%x\t", tempReadBuffer[frameHeadIndex + i]);
    //                 // }
    //                 std::cout << std::endl;
    //             }
    //             #endif
    //             frameHeadIndex += frameDataLength;
    //         }
    //     }
    // }

    // /**  
    // *   @brief      new thread to update motor feedback data
    //     Parameters:
    // *   @param      arg   	[in]this is a pointer to deliver parameter to thread
    // *   @return     none  
    //     */
    void* AmbotDriverCLASS::newReadMotorThread(void* arg)
    {
        AmbotDriverCLASS* ptr = (AmbotDriverCLASS*) arg;
        ptr->getAllMotorStateFromMCU();
        pthread_exit(0);
    }

    // /**  
    // *   @brief      new thread to update imu and force sensor data
    //     Parameters:
    // *   @param      arg   	[in]this is a pointer to deliver parameter to thread
    // *   @return     none  
    //     */
    // void* AmbotDriverCLASS::newReadSensorThread(void* arg)
    // {
    //     AmbotDriverCLASS* ptr = (AmbotDriverCLASS*) arg;
    //     ptr->getSensorDataFromMCU();
    //     pthread_exit(0);
    // }

    // /**  
    // *   @brief      create a receive data analysis thread
    //     Parameters:
    // *   @param      void
    // *   @return     none
    //     */
    void AmbotDriverCLASS::createReceiveThread(void)
    {
        if(pthread_create(&motorTid, NULL, newReadMotorThread, (void *)this) != 0)
            perror("Create read mcu data thread fail!\n");
        // if (sensorFd == -1)
        //     printf("No sensor serial port!!!\n");
        // else
        //     if(pthread_create(&sensorTid, NULL, newReadSensorThread, (void *)this) != 0)
        //         perror("Create sensor data read thread fail!\n");
    }

    /**  
    *   @brief      checkout all motor state, protect physical robot, special for motor 
        Parameters:
    *   @param      none
    *   @return     true:exception exist   false:everything normal
        */
    // bool AmbotDriverCLASS::checkAllMotorException(void)
    // {
    //     bool exceptionFlag = false;
    //     for (int i = 0; i < robotParams.motorNum; i++)
    //     {
    //         if (motorFB.at(i).state != 0)
    //         {
    //             exceptionFlag = true;
    //             std::cout << COUT_RED << "exception ID : " << motorFB.at(i).id << COUT_RESET << std::endl;
    //         }
    //     }
    //     return exceptionFlag;
    // }

    // /**  
    // *   @brief      update all Joint command in driver
    //     Parameters:
    // *   @param      in	    [in]the quote of joint command
    // *   @return     none
    //     */
    // void AmbotDriverCLASS::updateJointCmd(const ambot_msg::AmbotCommand& in)
    // {
    //     assert(in.Command.size() >= robotParams.jointMotorNum);
    //     if (robotParams.robotType == robotType.at(ambot_W1))
    //     {
    //         for (int i = 0; i < robotParams.motorNum; i++)
    //         {
    //             motorCommandSum.command.at(i).id = i+1;
    //             motorCommandSum.command.at(i).q = in.Command.at(i).q;
    //             motorCommandSum.command.at(i).dq = in.Command.at(i).dq;
    //             motorCommandSum.command.at(i).tor = in.Command.at(i).tor;
    //             motorCommandSum.command.at(i).kp = in.Command.at(i).Kp;
    //             motorCommandSum.command.at(i).kd = in.Command.at(i).Kd;
    //             if (motorCommandSum.command.at(i).id <= 0 || motorCommandSum.command.at(i).id > 16)
    //             {
    //                 std::cout << "update command id wrong, index:" << i << ", ID:" << motorCommandSum.command.at(i).id << std::endl;
    //             }
                
    //         }
    //     }else
    //     {
    //         for (int i = 0; i < robotParams.jointMotorNum; i++)
    //         {
    //             motorCommandSum.command.at(i).id = i+1;
    //             motorCommandSum.command.at(i).q = in.Command.at(i).q;
    //             motorCommandSum.command.at(i).dq = in.Command.at(i).dq;
    //             motorCommandSum.command.at(i).tor = in.Command.at(i).tor;
    //             motorCommandSum.command.at(i).kp = in.Command.at(i).Kp;
    //             motorCommandSum.command.at(i).kd = in.Command.at(i).Kd;
    //         }
    //     }
    // }

    // /**  
    // *   @brief      update all wheel velocity command in driver
    //     Parameters:
    // *   @param      in	    [in]the quote of wheel velocity command
    // *   @return     none
    //     */
    // void AmbotDriverCLASS::updateWheelCmd(const std::vector<float> &in)
    // {
    //     assert(in.size() <= robotParams.wheelMotorNum);
    //     for (int i = 0; i < in.size(); i++)
    //     {
    //         motorCommandSum.command.at(i + robotParams.jointMotorNum).id = i + 1 + robotParams.jointMotorNum;
    //         motorCommandSum.command.at(i + robotParams.jointMotorNum).q = 0;
    //         motorCommandSum.command.at(i + robotParams.jointMotorNum).dq = in.at(i);
    //         if(std::abs(motorCommandSum.command.at(i + robotParams.jointMotorNum).dq) < 0.01)
    //             motorCommandSum.command.at(i + robotParams.jointMotorNum).kd = 0.;
    //         else
    //             motorCommandSum.command.at(i + robotParams.jointMotorNum).kd = 4.;
    //         motorCommandSum.command.at(i + robotParams.jointMotorNum).tor = 0;
    //         motorCommandSum.command.at(i + robotParams.jointMotorNum).kp = 0;
    //     }
    // }

    // /**  
    // *   @brief      update exist motor command from all motor command
    //     Parameters:
    // *   @param      none	
    // *   @return     none
    //     */
    // void AmbotDriverCLASS::updateExistMotorCmd(void)
    // {
    //     assert(motorCommandExist.command.size() <= motorCommandSum.command.size());
    //     for (int i = 0; i < motorCommandExist.number; i++)
    //     {
    //         motorCommandExist.command.at(i) = motorCommandSum.command.at(motorIDExist.at(i) - 1);
    //     }
    // }

    // /**  
    // *   @brief      update exist wheel motor's feedback
    //     Parameters:
    // *   @param      none	
    // *   @return     none
    //     */
    // void AmbotDriverCLASS::updateWheelFb(void)
    // {
    //     assert(wheelVel.size() == robotParams.wheelMotorNum);
    //     for (int i = 0; i < wheelVel.size(); i++)
    //         wheelVel.at(i) = rosData.motorState.at(i + robotParams.jointMotorNum).vel;
    // }

    // /**  
    // *   @brief      open api for external get all wheel velocity 
    //     Parameters:
    // *   @param      out	    [out]the quote of all wheel motor velocity
    // *   @return     none
    //     */
    // void AmbotDriverCLASS::getWheelVelocity(std::vector<float> &out)
    // {
    //     assert(out.size() == robotParams.wheelMotorNum);
    //     out = wheelVel;
    // }

    // /**  
    // *   @brief      set position for all motor
    //     Parameters:
    // *   @param      position	[in]the quote of position for all motor
    // *   @return     true:get successful   false:get failed
    //     */
    // bool AmbotDriverCLASS::setMotorLocomotionCommand(const ambot_msg::AmbotCommand &command, const std::vector<float> &wheel)
    // {
    //     static protocolOutputBuffer_TP sendBuff;
    //     //update all motor command data
    //     updateJointCmd(command);
    //     updateExistMotorCmd();        
    //     protocol->createCommandFrame(protocol->FunctionCode.CONTROL, protocol->ControlCommandCode.SET_LOCOMOTION_CONTROL,
    //                                     motorCommandExist, sendBuff);
    //     if(sendBuff.length != txPacket(sendBuff))
    //         return false;
    //     // don't wait for set command response now 
    //     return true;
    // }

}


