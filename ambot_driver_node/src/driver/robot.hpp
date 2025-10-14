/*
 * robot.hpp
 * This is robot for yangtze electric company
 * Created on: march 1, 2024
 *      Author: chen chen
 */
#ifndef __ROBOT_HPP__
#define __ROBOT_HPP__

#include <vector>
#include "ambotDriver.hpp"
#include "ambotRosClass.hpp"
// #include "protocolStruct.hpp"
// 在 include/arm_driver_node/color_macros.hpp 或相关头文件中添加：
#ifndef COLOR_MACROS_HPP
#define COLOR_MACROS_HPP

// ANSI 颜色控制码
#define COUT_RESET   "\033[0m"
#define COUT_RED     "\033[31m"
#define COUT_GREEN   "\033[32m"
#define COUT_YELLOW  "\033[33m"
#define COUT_BLUE    "\033[34m"
#define COUT_MAGENTA "\033[35m"
#define COUT_CYAN    "\033[36m"
#define COUT_WHITE   "\033[37m"

#endif // COLOR_MACROS_HPP
class Robot
{
    private:
        /* data */
         ambot_driver_ns::RosClass *ros;
         ambot_driver_ns::AmbotDriverCLASS* robotDriver;
        // ambot_msg::AmbotCommand ambotCommand;

        std::vector<float> wheelVelCmd;
        std::thread spin_thread_;
        bool imuConnectFlag = false;
    public:
        Robot(int argc, char** argv);
        ~Robot();
        bool init(void);
        void runEnd(void);
        bool run(void);
        void setThreadRunFlag(void);
};
#endif