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
        // if (sensorFd == -1)
        //     printf("No sensor serial port!!!\n");
        // else
        //     if(pthread_create(&sensorTid, NULL, newReadSensorThread, (void *)this) != 0)
        //         perror("Create sensor data read thread fail!\n");
    }


}


