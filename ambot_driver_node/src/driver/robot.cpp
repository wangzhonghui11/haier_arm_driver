/*
 * robot.cpp
 * This is robot for yangtze electric company
 * Created on: march 1, 2024
 *      Author: chen chen
 */
#include "robot.hpp"
/**  
*   @brief      robot default construct function
    Parameters:
*   @param      
*   @return     none
    */
Robot::Robot(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    ros = new ambot_driver_ns::RosClass(argc, argv,"ambot_driver_node");
    // wheelVelCmd.resize(ros->robotFeatures.wheelMotorNum);
    spin_thread_ = std::thread([this]() {
        rclcpp::spin(ros->get_node_base_interface());
    });
}

/**  
*   @brief      robot deconstruct function
    Parameters:
*   @param      
*   @return     none
    */
Robot::~Robot()
{
    // wheelVelCmd.clear();
    // delete robotDriver;
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    delete ros;
    rclcpp::shutdown();
}

/**  
*   @brief      robot init function
    Parameters:
*   @param      
*   @return     none
    */
bool Robot::init(void)
{
    //1 init driver
    robotDriver = new ambot_driver_ns::AmbotDriverCLASS(ros->robotFeatures);
    if(robotDriver->initial())
        return true;
    else
        return false;
}

/**  
*   @brief      robot run end function
    Parameters:
*   @param      
*   @return     none
    */
void Robot::runEnd(void)
{
    // robotDriver->disableAllMotor();
}

/**  
*   @brief      robot run step
    Parameters:
*   @param      
*   @return     none
    */
bool Robot::run(void)
{
    if(rclcpp::ok())
    {
        // if(ros->terminate)
        // {
        //     ros->setTerminateValue();
        //     return false;                   //if return false, program will end, then will go to deconstruct function
        // }
        // ros->robotFbValuePub(robotDriver->rosData);
        // ros->getJointMotorCommand(ambotCommand);

        // // if (ros->robotFeatures.robotType  == robotDriver->robotType.at(ambot_W1))
        // //     ros->getWheelMotorCommand(wheelVelCmd);
        // if(!robotDriver->threadStop)
        // {
        //     robotDriver->setMotorLocomotionCommand(ambotCommand, wheelVelCmd);
        // }else
        // {
        //     ros->setTerminateValue();
        // }
        ros->rosSleep();
        return true; 
    }else
        return false; 

}

/**  
*   @brief      set thread stop flag
    Parameters:
*   @param      
*   @return     none
    */
void Robot::setThreadRunFlag(void)
{
    // robotDriver->threadStop = true;
}