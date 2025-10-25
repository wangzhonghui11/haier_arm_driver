/*  A class include yangtze river inspection robot hardware driver
    *
    * Author： 
    * Date: 2024-8-22
    * Email:
    *
*/

#include "ambotDriver.hpp"

namespace bimax_driver_ns{

    AmbotDriverCLASS::AmbotDriverCLASS(const std::shared_ptr<RosClass>& ros) : ros(ros)
    {
        protocol = new PrivateProtocolCLASS(ros);
        robotType.push_back("ambot_N1");
        robotType.push_back("ambot_W1");
        robotType.push_back("ambot_P1");
        // 1.open device file
        motorFd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY );
        if (motorFd == -1) {
            RCLCPP_ERROR(ros->get_logger(), "Failed to open motor serial port");
            // exit(0);
        }
        // 2.init serial config file
        struct termios SerialPortSettings;
        struct serial_struct ser_info;
        // 3.1 get serial current feature and save to SerialPortSettings
        tcgetattr(motorFd, &SerialPortSettings);
        // 3.2 according baud to set the serial baudrate
        if (ros->robotFeatures.motorDevBaud == 115200)
        {
            RCLCPP_ERROR(ros->get_logger(), "Failed to open motor serial port");
            cfsetispeed(&SerialPortSettings, B115200);
            cfsetospeed(&SerialPortSettings, B115200);
        }else if (ros->robotFeatures.motorDevBaud == 921600)
        {
            RCLCPP_ERROR(ros->get_logger(), "Failed to open motor serial port"); 
            cfsetispeed(&SerialPortSettings, B921600);
            cfsetospeed(&SerialPortSettings, B921600);
        }else if (ros->robotFeatures.motorDevBaud == 1000000)
        {
            RCLCPP_ERROR(ros->get_logger(), "Failed to open motor serial port");
            cfsetispeed(&SerialPortSettings, B1000000);
            cfsetospeed(&SerialPortSettings, B1000000);
        }
        else{
            RCLCPP_ERROR(ros->get_logger(), "Failed to open motor serial port");
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
        SerialPortSettings.c_cc[VTIME] = 5;         // timeout set, unit 1/10s, if set zero, will return right now 
        SerialPortSettings.c_cc[VMIN] = 1;        // wait for enough data to read, if set zero, data will read right now
        // 3.4 Enable linux FTDI low latency mode
        ioctl(motorFd, TIOCGSERIAL, &ser_info);
        ser_info.flags |= ASYNC_LOW_LATENCY;
        ioctl(motorFd, TIOCSSERIAL, &ser_info);
        // 3.5 Set the new attributes to the termios structure
        if((tcsetattr(motorFd, TCSANOW, &SerialPortSettings)) != 0) 
        {
            RCLCPP_ERROR(ros->get_logger(), "Failed to set serial port attributes! for port: %s", ros->robotFeatures.motorDevName.c_str());
            sleep(1);
        }
        else
            RCLCPP_INFO(ros->get_logger(), "Port %s open successfully", ros->robotFeatures.motorDevName.c_str());
    }
    
    AmbotDriverCLASS::~AmbotDriverCLASS()
    {
      delete  protocol;
    }
    
    ssize_t AmbotDriverCLASS::txPacket(protocolOutputBuffer_TP &out)
    {
        return  write(motorFd, out.buffer, out.length);
    }




    /**  
    *   @brief      initial function for MCU class
        Parameters:
    *   @param      void
    *   @return     none
        */
    bool AmbotDriverCLASS::initial(void)
    {
        threadStop = false;
        RCLCPP_INFO(ros->get_logger(), "Driver ready to init");
        createReceiveThread();
        return true;
    }


    // /**  
    // *   @brief      get all motor state feedback data
    //     Parameters:
    // *   @param      none
    // *   @return     true:get successful   false:get failed
    //     */

    // 全局变量用于计算频率
    static auto lastTime = std::chrono::steady_clock::now();
    static size_t totalBytes = 0;
    static double frequency = 0.0;
    ssize_t AmbotDriverCLASS::printReceivedDataWithFrequency(int motorFd) {
        uint8_t read_buffer[LEN_MAX];
        ssize_t currentReadCount = read(motorFd, read_buffer, LEN_MAX);
        
        if (currentReadCount > 0) {
            // 更新总字节数
            totalBytes += currentReadCount;
            
            // 计算时间差
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count() / 1000.0;
            
            // 计算频率（每秒字节数）
            if (elapsed > 0.1) {  // 至少0.1秒才更新频率，避免抖动
                frequency = totalBytes / elapsed;
                lastTime = now;
                totalBytes = 0;
            }
            
            // 打印接收到的数据
            std::cout << "Read " << currentReadCount << " bytes (";
            std::cout << std::fixed << std::setprecision(2) << frequency << " bytes/s):\n";
            
            // 打印十六进制数据
            for (ssize_t i = 0; i < currentReadCount; ++i) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') 
                        << static_cast<int>(read_buffer[i]) << " ";
            }
            std::cout << std::dec << "\n\n";
            return currentReadCount;
        }
}

    void AmbotDriverCLASS::printByteStream(const uint8_t* data, ssize_t count) {
            // 调试控制参数
            constexpr bool SHOW_ASCII = true;
            constexpr int BYTES_PER_LINE = 32;
            constexpr int MAX_LINES = 10;         
            if (count <= 0) return;        
            std::ios_base::fmtflags f(std::cout.flags()); // 保存格式状态          
            // 打印头信息
            std::cout << "\n▬▬▶ Read " << count << " bytes:\n";
            
            // 打印十六进制数据
            int lines_printed = 0;
            for (ssize_t i = 0; i < count && lines_printed < MAX_LINES; ++i) {
                // 行首地址标记
                if (i % BYTES_PER_LINE == 0) {
                    std::cout << "0x" << std::setw(4) << std::setfill('0') 
                            << std::hex << i << ": ";
                }                
                // 十六进制打印
                std::cout << std::setw(2) << std::setfill('0') 
                        << static_cast<int>(data[i]) << " ";            
                // 行尾处理
                if ((i+1) % BYTES_PER_LINE == 0 || i == count-1) {
                    if (SHOW_ASCII) {
                        // 对齐填充
                        for (int j = 0; j < BYTES_PER_LINE - (i % BYTES_PER_LINE + 1); ++j) 
                            std::cout << "   ";
                        
                        // ASCII展示
                        std::cout << "| ";
                        for (int j = i - (i % BYTES_PER_LINE); j <= i; ++j) {
                            char c = data[j];
                            std::cout << (isprint(c) ? c : '.');
                        }
                    }
                    std::cout << "\n";
                    lines_printed++;
                }
            }            
            if (count > MAX_LINES * BYTES_PER_LINE) {
                std::cout << "[... " << (count - MAX_LINES * BYTES_PER_LINE) 
                        << " bytes omitted ...]\n";
            }
            
            std::cout.flags(f); // 恢复格式状态
        }
    void AmbotDriverCLASS::getAllMotorStateFromMCU(void)
    {   
        // 1. init all local variables
        uint8_t inFlag;
        uint8_t currentReadCount;
        uint8_t read_buffer[LEN_MAX];
        // 2. set init value to variables
        // 3.wait for receive enough data to analysis
        for(;;)
        {
            // 4. if detect ctrl+C, current thread will exit 
            if (threadStop)
            {
                RCLCPP_INFO(ros->get_logger(), "GetAllMotorStateFromMCU thread exit");
                pthread_exit(NULL);
            }
            //100hz
            std::this_thread::sleep_for(std::chrono::milliseconds(30));  // 20ms
        //    currentReadCount=printReceivedDataWithFrequency(motorFd);
            currentReadCount = read(motorFd, read_buffer,LEN_MAX);
            if (currentReadCount > 0) {
            //     // 调试模式才打印原始数据
                #ifdef DEBUG_MODE
                printByteStream(read_buffer, currentReadCount);
                #endif
                // 协议处理（自动包含帧格式打印）
             protocol->processFrame(read_buffer,currentReadCount); 
            } 
        }
        
    }

    void AmbotDriverCLASS::ProccessAllMotorStateFromMCU(void){
        for(;;)
       {

           //50hz
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 20ms
           if (threadStop)
            {
                RCLCPP_INFO(ros->get_logger(), "ProccessAllMotorStateFromMCU thread exit");
                pthread_exit(NULL);
            }
            protocol->updateDataConsumer(mecarm,lifter_l_pos,lifter_r_pos,jaw_pos);
       }
    }
    void AmbotDriverCLASS::floatToUint32(float input, uint8_t* des) {
        uint32_t bits;
        memcpy(&bits, &input, sizeof(float));

        // 小端序存储（低字节在前）
        des[0] = bits & 0xFF;          // 最低字节
        des[1] = (bits >> 8) & 0xFF;
        des[2] = (bits >> 16) & 0xFF;
        des[3] = (bits >> 24) & 0xFF;  // 最高字节
    }
    void* AmbotDriverCLASS::newReadMotorThread(void* arg)
    {
        AmbotDriverCLASS* ptr = (AmbotDriverCLASS*) arg;
        ptr->getAllMotorStateFromMCU();
        pthread_exit(0);
    }
    void* AmbotDriverCLASS::newProccessMotorThread(void* arg)
    {
        AmbotDriverCLASS* ptr = (AmbotDriverCLASS*) arg;
        ptr->ProccessAllMotorStateFromMCU();
        pthread_exit(0);
    }
    void AmbotDriverCLASS::createReceiveThread(void)
    {
        if(pthread_create(&motorTid, NULL, newReadMotorThread, (void *)this) != 0)
            perror("Create read mcu data thread fail!\n");
        if(pthread_create(&sensorTid, NULL, newProccessMotorThread, (void *)this) != 0)
            perror("Create read mcu data thread fail!\n");
    }
    bool AmbotDriverCLASS::LifterMotorprocess(bimax_msgs::msg::RobotCommand& cmd)
    {
        uint8_t des_left[4];
        uint8_t des_right[4];
        floatToUint32(cmd.motor_command[0].q, des_left);  // 填充 des_left
        floatToUint32(cmd.motor_command[4].q, des_right); // 填充 des_right
        uint8_t databuf[8];
        memcpy(databuf, des_left, 4);      // 前 4 字节 = des_left
        memcpy(databuf + 4, des_right, 4); // 后 4 字节 = des_right
        // std::cout << "databuf: ";
        // for (int i = 0; i < 8; i++) {
        //     std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') 
        //             << static_cast<int>(databuf[i]) << " ";
        // }
        // std::cout << std::endl;
        cmdframLiftsSet.databuf=databuf;
        return setMotorLocomotionCommand(cmdframGroup[STORE_NUM_LIFTS_SET]);

    }
    bool AmbotDriverCLASS::YiyouMotorprocess(bimax_msgs::msg::RobotCommand& cmd) {
        uint8_t databuf[28] = {0};  // 初始化全0

        // 1. 保留前3字节（reserver1 + reserver2）
        databuf[0] = 0;  // reserver1的低字节
        databuf[1] = 0;  // reserver1的高字节
        databuf[2] = 0;  // reserver2

        // 2. 存储 Motor_Control_Mode（第4字节）
        databuf[3] = static_cast<uint8_t>(cmd.motor_command[1].mode);

        // 3. 填充电机位置数据（从第5字节开始）
        if (databuf[3] == 0 || databuf[3] == 1) {  // 检查模式有效性
            const int offsets[] = {4, 8, 12, 16, 20, 24};  // 对应 position_motor1~6 的偏移量
            const int indices[] = {1, 2, 3, 5, 6, 7};      // motor_command 的索引

            for (int i = 0; i < 6; i++) {
                // 安全转换浮点数到字节流（小端）
                union {
                    float f;
                    uint8_t b[4];
                } converter;
                converter.f = cmd.motor_command[indices[i]].q;
                memcpy(databuf + offsets[i], converter.b, 4);
            }
        }

        // 4. 调试输出（十六进制）
        std::cout << "databuf (hex): ";
        for (int i = 0; i < 28; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') 
                    << static_cast<int>(databuf[i]) << " ";
        }
        std::cout << std::dec << std::endl;

        // 5. 提交命令
        memcpy(cmdframMecArmSet.databuf, databuf, sizeof(databuf));
        return setMotorLocomotionCommand(cmdframGroup[STORE_NUM_MECARM_SET]);
    }
    bool AmbotDriverCLASS::JawCommandProcess(float pos)
    {
        uint8_t jaw_pos[4];
        floatToUint32(pos, jaw_pos);  // 填充 des_left
        cmdframJawSet.databuf=jaw_pos;
        return setMotorLocomotionCommand(cmdframGroup[STORE_NUM_JAW_SET]);
    }
    bool AmbotDriverCLASS::CommandFrameProcess(bimax_msgs::msg::RobotCommand& cmd)
    {
        //LifterMotorprocess(cmd);
        YiyouMotorprocess(cmd);
        return true;
    }
    bool AmbotDriverCLASS::CommandServeLedProcess(uint8_t green,uint8_t yellow)
    {
        uint8_t led[2]={green,yellow};
        cmdframLedSet.databuf=led;
        setMotorLocomotionCommand(cmdframGroup[STROE_NUM_LED_SET]);
        return true;
    }
    bool AmbotDriverCLASS::CommandServeCatcherProcess(uint8_t catcher_gear,uint8_t catcher_state)
    {
        uint8_t catcher[4]={0x00,0x00,catcher_gear,catcher_state};
        cmdframCatcherSet.databuf=catcher;
        setMotorLocomotionCommand(cmdframGroup[STORE_NUM_CATCHER_SET]);
        return true;
    }
    bool AmbotDriverCLASS::CommandServeMopProcess(uint16_t mop_motor_pwm,uint8_t mop_state)
    {
        uint8_t catcher[3]={mop_motor_pwm & 0xFF,(mop_motor_pwm >> 8) & 0xFF,mop_state};
        cmdframMopSet.databuf=catcher;
        setMotorLocomotionCommand(cmdframGroup[STORE_NUM_MOP_SET]);
        return true;
    }
    bool AmbotDriverCLASS::CommandServeMagnetProcess(uint8_t left_magnet_state,uint8_t right_magnet_state)
    {
        uint8_t magnet[2]={left_magnet_state,right_magnet_state};
        cmdframMagnetet.databuf=magnet;
        setMotorLocomotionCommand(cmdframGroup[STORE_NUM_MAGNET_SET]);
        return true;
    }

    bool AmbotDriverCLASS::setMotorLocomotionCommand(CommFrame* frame)
    {
        protocolOutputBuffer_TP sendBuff;
        uint8_t cmdId=frame->cmd_ID;
        switch(cmdId)
        {
            case ID_CMD_TIME_SET://设置MCU时间
                sendBuff.length=protocol->comm_frame_upload(cmdframGroup[STORE_NUM_TIME_SET],sendBuff.buffer);	
                    break;

            case ID_CMD_MOP_SET://拖布电机控制
                sendBuff.length=protocol->comm_frame_upload(cmdframGroup[STORE_NUM_MOP_SET],sendBuff.buffer);	
                    break; 

            case ID_CMD_JAW_SET://夹爪电机控制
                sendBuff.length=protocol->comm_frame_upload(cmdframGroup[STORE_NUM_JAW_SET],sendBuff.buffer);	
                    break;

            case ID_CMD_CATCHER_SET://吸尘电机控制
                sendBuff.length=protocol->comm_frame_upload(cmdframGroup[STORE_NUM_CATCHER_SET],sendBuff.buffer);					 
                    break;
                    
            case ID_CMD_MAGNET_SET: //磁铁控制
                sendBuff.length=protocol->comm_frame_upload(cmdframGroup[STORE_NUM_MAGNET_SET],sendBuff.buffer);			
                    break;
            
            case ID_CMD_MECARM_SET://机械臂控制	
                sendBuff.length=protocol->comm_frame_upload(cmdframGroup[STORE_NUM_MECARM_SET],sendBuff.buffer);			
                    break;         
            case ID_CMD_LIFTS_SET://升降电机控制
                sendBuff.length=protocol->comm_frame_upload(cmdframGroup[STORE_NUM_LIFTS_SET],sendBuff.buffer);			
                    break; 
            case ID_CMD_LED_SET: //LED灯控制
                 sendBuff.length=protocol->comm_frame_upload(cmdframGroup[STROE_NUM_LED_SET],sendBuff.buffer);			 
                    break;
            case ID_CMD_HEARTBEAT:
                    break;			
            }
        if(sendBuff.length != txPacket(sendBuff)){
            RCLCPP_INFO(ros->get_logger(), "SetMotorLocomotionCommand failed to send");
            return false;
        }
        else{
            RCLCPP_INFO(ros->get_logger(), "SetMotorLocomotionCommand succeed to send");
        }
        // don't wait for set command response now 
        return true;
    }

}


