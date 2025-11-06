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
        struct termios serialPortSettings;
        struct serial_struct serInfo;

        protocol = new PrivateProtocolCLASS(ros);
        motorFd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY );
        if (motorFd == -1) {
            RCLCPP_ERROR(ros->get_logger(), "Failed to open motor serial port");
            // exit(0);
        }
        tcgetattr(motorFd, &serialPortSettings);
        if (ros->motor_baud == 115200)
        {
            RCLCPP_INFO(ros->get_logger(), "set baud: 115200 successfully!");
            cfsetispeed(&serialPortSettings, B115200);
            cfsetospeed(&serialPortSettings, B115200);
        }else if (ros->motor_baud == 921600)
        {
            RCLCPP_INFO(ros->get_logger(), "set baud: 921600 successfully!");
            cfsetispeed(&serialPortSettings, B921600);
            cfsetospeed(&serialPortSettings, B921600);
        }else if (ros->motor_baud== 1000000)
        {
            RCLCPP_INFO(ros->get_logger(), "set baud: 1000000 successfully!");
            cfsetispeed(&serialPortSettings, B1000000);
            cfsetospeed(&serialPortSettings, B1000000);
        }
        else{
            RCLCPP_INFO(ros->get_logger(), "set defalut baud: 1000000 successfully!");
            cfsetispeed(&serialPortSettings, B1000000);
            cfsetospeed(&serialPortSettings, B1000000);
        }
        // 3.3 set uart communicate features
        serialPortSettings.c_cflag &= ~PARENB;      // Disable parity
        serialPortSettings.c_cflag &= ~CSTOPB;      // 1 stop bit
        serialPortSettings.c_cflag |= CS8;          // 8 bits per byte
        serialPortSettings.c_cflag &= ~CRTSCTS;     // Disable RTS/CTS hardware flow control
        serialPortSettings.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = serialPortSettings
        serialPortSettings.c_lflag &= ~ICANON;      // Disable canonical mode
        serialPortSettings.c_lflag &= ~ECHO;        // Disable echo
        serialPortSettings.c_lflag &= ~ECHOE;       // Disable erasure
        serialPortSettings.c_lflag &= ~ECHONL;      // Disable new-line echo
        serialPortSettings.c_lflag &= ~ISIG;        // Disable interpretation of INTR, QUIT and SUSP
        serialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        serialPortSettings.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
        serialPortSettings.c_oflag &= ~OPOST;       // Prevent special interpretation of output bytes (e.g. newline chars)
        serialPortSettings.c_oflag &= ~ONLCR;       // Prevent conversion of newline to carriage return/line feed
        serialPortSettings.c_cc[VTIME] = 0;         // timeout set, unit 1/10s, if set zero, will return right now 
        serialPortSettings.c_cc[VMIN] = 0;        // wait for enough data to read, if set zero, data will read right now
        // 3.4 Enable linux FTDI low latency mode
        ioctl(motorFd, TIOCGSERIAL, &serInfo);
        serInfo.flags |= ASYNC_LOW_LATENCY;
        ioctl(motorFd, TIOCSSERIAL, &serInfo);
        // 3.5 Set the new attributes to the termios structure
        if((tcsetattr(motorFd, TCSANOW, &serialPortSettings)) != 0) 
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
        size_t buffer_len_ = 0;     // 当前缓冲区数据长度
         uint8_t temp_buf[1024];
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

            int n = read(motorFd, temp_buf, sizeof(temp_buf));

            if (n > 0) {
                // 将新数据添加到缓冲区
                if (buffer_len_ + n <= sizeof(read_buffer)) {
                    memcpy(read_buffer + buffer_len_, temp_buf, n);
                    buffer_len_ += n;
                    
                    // 处理完整的数据帧，并获取已处理的字节数
                    uint16_t processed_bytes = protocol->processFrame(read_buffer, buffer_len_);
                    
                    // 关键步骤：从缓冲区移除已处理的数据
                    if (processed_bytes > 0) {
                        if (processed_bytes < buffer_len_) {
                            // 将未处理的数据移动到缓冲区开头
                            memmove(read_buffer, read_buffer + processed_bytes, buffer_len_ - processed_bytes);
                        }
                        buffer_len_ -= processed_bytes;  // 更新缓冲区长度
                    }
                    
                } else {
                    // 缓冲区溢出，清空重新开始
                    buffer_len_ = 0;
                    std::cerr << "Serial buffer overflow!" << std::endl;
                }
            } else if (n == 0) {
                // 无数据可读，短暂休眠
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } else {
                // 读取错误
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    std::cerr << "Serial read error: " << strerror(errno) << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
         // std::this_thread::sleep_for(std::chrono::milliseconds(20));  // 20ms
            // currentReadCount = read(motorFd, read_buffer,LEN_MAX);
            // if (currentReadCount > 0) {
            // //     // 调试模式才打印原始数据
            //     #ifdef DEBUG_MODE
            //     printByteStream(read_buffer, currentReadCount);
            //     #endif
            //     printByteStream(read_buffer, currentReadCount);
            //  protocol->processFrame(read_buffer,currentReadCount); 
            // } 
            // else{
            //     RCLCPP_ERROR(ros->get_logger(), "MUC uart date is update failed!");
            // }
        
    }

    void AmbotDriverCLASS::proccessAllMotorStateFromMCU(void){
        static auto last_states_time = std::chrono::steady_clock::now();
        for(;;)
       {
           if (threadStop)
            {
                RCLCPP_INFO(ros->get_logger(), "proccessAllMotorStateFromMCU thread exit");
                pthread_exit(NULL);
            }
            float lifter_right_value = getLifterRightPos();
            float lifter_left_value = getLifterLeftPos();
            float jaw_value = getJawPos();
            protocol->updateDataConsumer(mecarm,lifter_left_value,lifter_right_value,jaw_value);
            setLifterRightPos(lifter_right_value);
            setLifterLeftPos(lifter_left_value);
            setJawPosition(jaw_value);
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_states_time).count() >= 1000) 
            {

                checkToMotorStates(mecarm);
                last_states_time=now;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(3));  // 20ms
       }
    }
    void AmbotDriverCLASS::checkToMotorStates(YiyouMecArm &mecarm)
    {
        if(mecarm.status_motor1==255)
        {
            RCLCPP_WARN(ros->get_logger(), "Motor1 find failed,plase check motor_power!");
        }
        if(mecarm.status_motor2==255)
        {
            RCLCPP_WARN(ros->get_logger(), "Motor2 find failed,plase check motor_power!");
        }
        if(mecarm.status_motor3==255)
        {
            RCLCPP_WARN(ros->get_logger(), "Motor4 find failed,plase check motor_power!");
        }
        if(mecarm.status_motor4==255)
        {
            RCLCPP_WARN(ros->get_logger(), "Motor5 find failed,plase check motor_power!");
        }
        if(mecarm.status_motor5==255)
        {
            RCLCPP_WARN(ros->get_logger(), "Motor6 find failed,plase check motor_power!");
        }

        if(mecarm.error_motor1==34321)
        {
            RCLCPP_WARN(ros->get_logger(), "Motor1 pos error very big ,plase clean error!");
        }
        if(mecarm.status_motor2==34321)
        {
            RCLCPP_WARN(ros->get_logger(), "Motor2 pos error very big ,plase clean error!");
        }

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
        ptr->proccessAllMotorStateFromMCU();
        pthread_exit(0);
    }
    void AmbotDriverCLASS::createReceiveThread(void)
    {
        if(pthread_create(&motorTid, NULL, newReadMotorThread, (void *)this) != 0)
            perror("Create read mcu data thread fail!\n");
        if(pthread_create(&sensorTid, NULL, newProccessMotorThread, (void *)this) != 0)
            perror("Create read mcu data thread fail!\n");
    }
    bool AmbotDriverCLASS::commandFrameProcess(bimax_msgs::msg::RobotCommand& cmd)
    {
        static int lifter_counter = 0;
        static int lifter_threshold = 2;  // Lifter累计2次执行
        lifter_counter++;
        if (lifter_counter >= lifter_threshold) {
            lifterMotorProcess(cmd);
            lifter_counter = 0;  // 重置计数器
        }     
        yiyouMotorProcess(cmd);      
        return true;        
    }

    bool AmbotDriverCLASS::setMotorLocomotionCommand(CommFrame* frame)
    {
        protocolOutputBuffer_TP sendBuff;

        sendBuff.length=protocol->commFrameUpload(frame,sendBuff.buffer);	

        if(sendBuff.length != txPacket(sendBuff)){
            RCLCPP_ERROR(ros->get_logger(), "SetMotorLocomotionCommand failed to send");
            return false;
        }
        else{
            RCLCPP_DEBUG(ros->get_logger(), "SetMotorLocomotionCommand succeed to send");
        }
        // don't wait for set command response now 
        return true;
    }
    bool AmbotDriverCLASS::lifterMotorProcess(bimax_msgs::msg::RobotCommand& cmd)
    {
        LiftsSet dataLiftsSet = {0};
        dataLiftsSet.real.left_pos=cmd.motor_command[0].q;
        dataLiftsSet.real.right_pos=cmd.motor_command[4].q;       
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
        return setMotorLocomotionCommand(&cmdframLiftsSet);
    }
    bool AmbotDriverCLASS::yiyouMotorProcess(bimax_msgs::msg::RobotCommand& cmd) 
    {
        if(cmd.motor_command[1].mode==0||cmd.motor_command[1].mode==1){
            MecArmSet dataMecArmSet = {0};
            dataMecArmSet.real.Motor_Control_Mode=static_cast<uint8_t>(cmd.motor_command[1].mode);
            dataMecArmSet.real.position_motor1=cmd.motor_command[1].q;
            dataMecArmSet.real.position_motor2=cmd.motor_command[2].q;
            dataMecArmSet.real.position_motor3=cmd.motor_command[3].q;
            dataMecArmSet.real.position_motor4=cmd.motor_command[5].q;
            dataMecArmSet.real.position_motor5=cmd.motor_command[6].q; 
            dataMecArmSet.real.position_motor6=cmd.motor_command[7].q;                       
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
            RCLCPP_INFO(ros->get_logger(), "cmd.motor_command[1].q = %f ", cmd.motor_command[1].q);
            RCLCPP_INFO(ros->get_logger(), "cmd.motor_command[2].q = %f ", cmd.motor_command[5].q);   
            return setMotorLocomotionCommand(&cmdframMecArmSet);
        }
        else if(cmd.motor_command[1].mode == 3) {
            // 处理模式3（完整控制模式）
            MecArmMitSet command = {0};
            
            // 填充保留字段
            command.real.reserver1 = 0;
            command.real.reserver2 = 0;
            
            // 设置控制模式
            command.real.Motor_Control_Mode = static_cast<uint8_t>(cmd.motor_command[1].mode);
            
            // 填充电机1数据
            command.real.cmd_pos1 = cmd.motor_command[1].q;
            command.real.cmd_vel1 = cmd.motor_command[1].dq;    // 假设qd是速度
            command.real.cmd_torque1 = cmd.motor_command[1].tau; // 假设tau是扭矩
            command.real.kp1 = cmd.motor_command[1].kp;         // 假设kp是比例增益
            command.real.kd1 = cmd.motor_command[1].kd;         // 假设kd是微分增益
            
            // 填充电机2数据
            command.real.cmd_pos2 = cmd.motor_command[2].q;
            command.real.cmd_vel2 = cmd.motor_command[2].dq;
            command.real.cmd_torque2 = cmd.motor_command[2].tau;
            command.real.kp2 = cmd.motor_command[2].kp;
            command.real.kd2 = cmd.motor_command[2].kd;
            CommFrame cmdArmMitSet = {
                .head_H =FRAME_HEAD_H ,
                .head_L = FRAME_HEAD_L,
                .frame_ID_H = 0,
                .frame_ID_L = 0,
                .length = 1 + DATA_LEGTH_MECARM_MIT_SET,
                .cmd_ID =  ID_CMD_MECARM_MIT_SET,
                .databuf = command.data,
                .CRC_H = 0,
                .CRC_L = 0
            };        
            return setMotorLocomotionCommand(&cmdArmMitSet);
        }   
    }
    bool AmbotDriverCLASS::jawCommandProcess(float pos)
    {
        JawSet dataJawSet = {0};
        dataJawSet.real.jaw_motor_angle=pos;
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
        return setMotorLocomotionCommand(&cmdframJawSet);
    }
   bool AmbotDriverCLASS::commandSetTimeProcess(uint64_t time)
    {
        TimeSet command = {0};
        command.real.time=time;
        CommFrame TimeSet = {
            .head_H =FRAME_HEAD_H ,
            .head_L = FRAME_HEAD_L,
            .frame_ID_H = 0,
            .frame_ID_L = 0,
            .length = 1 + DATA_LEGTH_TIME_SET,
            .cmd_ID =  ID_CMD_TIME_SET,
            .databuf = command.data,
            .CRC_H = 0,
            .CRC_L = 0
        };
        setMotorLocomotionCommand(&TimeSet);
        return true;
    }
    bool AmbotDriverCLASS::commandServeLedProcess(uint8_t green,uint8_t yellow)
    {
        LedSet dataLedSet = {0};
        dataLedSet.real.led_green_state=green;
        dataLedSet.real.led_yellow_state=yellow;   
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
        setMotorLocomotionCommand(&cmdframLedSet);
        return true;
    }
    bool AmbotDriverCLASS::commandServeCatcherProcess(uint8_t catcher_gear,uint8_t catcher_state)
    {
        CatcherSet dataCatcherSet = {0};
        dataCatcherSet.real.catcher_gear=catcher_gear;
        dataCatcherSet.real.catcher_state=catcher_state;      
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
        setMotorLocomotionCommand(&cmdframCatcherSet);
        return true;
    }
    bool AmbotDriverCLASS::commandServeMopProcess(uint16_t mop_motor_pwm,uint8_t mop_state)
    {
        MopSet dataMopSet = {0};
        dataMopSet.real.mop_motor_pwm=mop_motor_pwm;
        dataMopSet.real.mop_state=mop_state;       
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
        setMotorLocomotionCommand(&cmdframMopSet);
        return true;
    }
    bool AmbotDriverCLASS::commandServeMagnetProcess(uint8_t left_magnet_state,uint8_t right_magnet_state)
    {
        MagnetSet dataMagnetSet = {0};
        dataMagnetSet.real.left_magnet_state=left_magnet_state;
        dataMagnetSet.real.right_magnet_state=right_magnet_state;     
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
        setMotorLocomotionCommand(&cmdframMagnetet);
        return true;
    }
    void AmbotDriverCLASS::setLifterLeftPos(float new_pos) {      
        lifterLiftPos.store(new_pos, std::memory_order_release);
    }
    void AmbotDriverCLASS::setLifterRightPos(float new_pos) {     
        lifterRightPos.store(new_pos, std::memory_order_release);
    }

}


