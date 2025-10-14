#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/time.h>
#include "robot.hpp"
#include <iostream>
/*register signal handler to deal ctrl+C, need a class pointer*/
Robot* myRobot;

/* add get interrupt signal of SIGINT */
void signalHandler(int signum) 
{
    myRobot->setThreadRunFlag();
    usleep(400000);
    std::cout << COUT_RED << "Received signal: " << signum << ", robot drive program will exit()." << COUT_RESET << std::endl;
    // 调用类的成员函数
    myRobot->runEnd();
    exit(0);
}

int main(int argc, char** argv)
{
    Robot robot(argc, argv);
    if(robot.init())
    {
        //2 register exception signal get handler 
        myRobot = &robot;
        signal(SIGINT, signalHandler);
        while (robot.run()){}
    }
    return 0;
}
