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
    ros = std::make_shared<bimax_driver_ns::RosClass>(argc, argv, "ambot_driver_node");
    spin_thread_ = std::thread([this]() {
    rclcpp::spin(ros->get_node_base_interface());});
}

/**  
*   @brief      robot deconstruct function
    Parameters:
*   @param      
*   @return     none
    */
Robot::~Robot()
{

    delete robotDriver;
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    rclcpp::shutdown();
}

rclcpp::Logger Robot::get_logger(void) const
{
    return ros->get_logger();
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
    robotDriver = new bimax_driver_ns::AmbotDriverCLASS(ros);
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

        // 初始化时间记录
    static auto last_states_time = std::chrono::steady_clock::now();
    static auto last_control_time = std::chrono::steady_clock::now();
    static auto last_tools_time = std::chrono::steady_clock::now();
     if(rclcpp::ok())
    {

        auto now = std::chrono::steady_clock::now();
        // 任务0：电机反馈（20ms周期）
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_states_time).count() >= 20) 
        {
            ros->robotFbValuePub(robotDriver->mecarm,robotDriver->lifter_l_pos,robotDriver->lifter_r_pos,robotDriver->jaw_pos);
            last_states_time=now;
        }
        // 任务1：电机控制（10ms周期）
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_control_time).count() >= 10) 
        {
            bool jaw_changed=ros->get_jaw_cmd(jaw_cmd); 
            if(ros->getJointMotorCommand(CommandValues)) 
            {
                robotDriver->CommandFrameProcess(CommandValues);
            }            
            if(jaw_changed)
           {
                 robotDriver->JawCommandProcess(jaw_cmd);
            }  
            last_control_time = now;  // 更新时间戳
        }

        // 任务2：LED控制（100ms周期） 
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tools_time).count() >= 200) 
        {
            bool green_changed = ros->get_green_state(rad_led);
            bool yellow_changed = ros->get_yellow_state(yellow_led);
            bool left_magnet_changed =ros->get_left_magnet_state(left_magnet_state);
            bool right_magnet_changed =ros->get_right_magnet_state(right_magnet_state);           
            bool catcher_gear_changed =ros->get_catcher_gear_state(catcher_gear);
            bool catcher_state_changed =ros->get_catcher_state(catcher_state);  
            bool mop_motor_pwm_changed =ros->get_mop_motor_pwm_state(mop_motor_pwm);
            bool mop_state_changed =ros->get_mop_state(mop_state); 

            if(left_magnet_changed||right_magnet_changed)
           {
                robotDriver->CommandServeMagnetProcess(left_magnet_state, right_magnet_state);
            }
            if(green_changed||yellow_changed)
            {

                robotDriver->CommandServeLedProcess(rad_led, yellow_led);
            }
            if(catcher_gear_changed||catcher_state_changed)
            {

                robotDriver->CommandServeCatcherProcess(catcher_gear, catcher_state);
            }
            if(mop_motor_pwm_changed||mop_state_changed)
            {

                robotDriver->CommandServeMopProcess(mop_motor_pwm, mop_state);
            }
            last_tools_time = now;  // 更新时间戳
        }
 
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
    robotDriver->threadStop = true;
}