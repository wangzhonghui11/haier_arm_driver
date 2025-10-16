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
        protocol = new PrivateProtocolCLASS();


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
        SerialPortSettings.c_cc[VTIME] = 5;         // timeout set, unit 1/10s, if set zero, will return right now 
        SerialPortSettings.c_cc[VMIN] = 1;        // wait for enough data to read, if set zero, data will read right now
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

        createReceiveThread();
        return true;
    }


    // /**  
    // *   @brief      get all motor state feedback data
    //     Parameters:
    // *   @param      none
    // *   @return     true:get successful   false:get failed
    //     */
     #define TEMP_READ_BUFFER_LENGTH 256*8
     #define LEN_MAX 4096
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
        int16_t frameHeadIndex, currentTail;
        uint8_t tempReadBuffer[TEMP_READ_BUFFER_LENGTH];
        uint8_t motorFeedbackBuffer[TEMP_READ_BUFFER_LENGTH];
        uint8_t read_buffer[LEN_MAX];
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
            usleep(100000);
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
            // for (ssize_t i = 0; i < currentReadCount; ++i) {
            //     std::cout << std::hex << std::setw(2) << std::setfill('0') 
            //             << static_cast<int>(read_buffer[i]) << " ";
            // }
             //protocol->processFrame(read_buffer,currentReadCount);      
}
        
    }


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
    // *   @brief      create a receive data analysis thread
    //     Parameters:
    // *   @param      void
    // *   @return     none
    //     */
    void AmbotDriverCLASS::createReceiveThread(void)
    {
        if(pthread_create(&motorTid, NULL, newReadMotorThread, (void *)this) != 0)
            perror("Create read mcu data thread fail!\n");
    }


}


