#ifndef __ROS_CLASS_YANGTZE_HPP__
#define __ROS_CLASS_YANGTZE_HPP__

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "queue.hpp"
#include <cassert>
#include "protocolStruct.hpp"
#include "bimax_msgs/msg/motor_command.hpp"
#include "bimax_msgs/msg/motor_state.hpp"
#include "bimax_msgs/msg/robot_command.hpp"
#include "bimax_msgs/msg/robot_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "bimax_msgs/msg/motor_error.hpp"
#include "std_msgs/msg/float32.hpp"
#include "bimax_msgs/srv/led_control.hpp"  
#include "bimax_msgs/srv/magnet_control.hpp"  
#include "bimax_msgs/srv/catcher_control.hpp"
#include "bimax_msgs/srv/mop_control.hpp"    
namespace bimax_driver_ns
{
        typedef struct 
    {
        std::string robotType;
        std::string motorDevName;
        std::string sensorDevName;
        unsigned int motorDevBaud;
        unsigned int sensorDevBaud;
        uint8_t jointMotorNum;
        uint8_t wheelMotorNum;
        uint8_t motorNum;
        std::vector<uint8_t> motorControlMode;
        std::vector<float> motorMaxAngle;
        std::vector<float> motorMinAngle;
        std::vector<float> motorVelocity;
        std::vector<float> motorOffset;
        std::vector<float> motorAxisDirection;
        uint8_t forceDataNum;
        uint8_t sensorNum;
        uint16_t rosRate;
        float   carMaxVel;
        float   wheelRadius;
    }RobotDriver_TP;

    using namespace std;

    class RosClass : public rclcpp::Node
    {
    public:
        /* terminate flag */
        bool terminate = false;

        /* all ros parameters server:robot params */
        /* construct and deconstruct function */
        RosClass(int argc, char** argv, const std::string& node_name = "yangtze_node");
        ~RosClass();
        RobotDriver_TP robotFeatures;
        /* open function api */
        bool init();
        void rosSleep();

        bool getJointMotorCommand(bimax_msgs::msg::RobotCommand& msg);
        void getParameters(map<string, float>& robot_params, map<string, string>& robot_devices);

        void jawCallback(const std_msgs::msg::Float32::SharedPtr msg) ;
        void commandCallback(const bimax_msgs::msg::RobotCommand::SharedPtr msg) ;
        bool get_green_state(uint8_t  &green_state_) ;
        bool get_yellow_state(uint8_t  &yellow_state_);  
        bool get_right_magnet_state(uint8_t  &right_magnet_state) ;
        bool get_left_magnet_state(uint8_t  &left_magnet_state) ;
        bool get_catcher_gear_state(uint8_t  &catcher_gear_state);
        bool get_catcher_state(uint8_t  &catcher_state);
        bool get_mop_state(uint8_t  &mop_state) ;
        bool get_mop_motor_pwm_state(uint16_t  &mop_motor_pwm_state);
        bool get_jaw_cmd(float  &jaw_cmd); 
        void robotFbValuePub(YiyouMecArm &mecarm,const float lef ,const float righ,const float jaw_pos);

        std::string motor_device;
        int motor_baud;
    private:
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr terminateValuePub;
        std::queue<bimax_msgs::msg::RobotCommand> command_queue_;
        std::mutex queue_mutex_;
        std::condition_variable queue_cv_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr terminateValueSub;
        bimax_msgs::msg::RobotCommand latest_command_;
        std::atomic<bool> command_ready_{false};
        float last_jaw_cmd_;
        float jaw_cmd_;
        uint8_t last_mop_state_;
        uint16_t last_mop_motor_pwm_state_;
        uint8_t mop_state_;
        uint16_t mop_motor_pwm_state_;
        uint8_t last_catcher_gear_state_;   // 新增：记录上一次状态
        uint8_t last_catcher_state_;  // 新增
        uint8_t catcher_gear_state_;   
        uint8_t catcher_state_;  // 新增
        uint8_t last_green_state_;   // 新增：记录上一次状态
        uint8_t last_yellow_state_;  // 新增
        uint8_t green_state_;   // 存储绿灯状态
        uint8_t yellow_state_; // 存储黄灯状态
        uint8_t last_left_magnet_state_;   // 新增：记录上一次状态
        uint8_t last_right_magnet_state_;  // 新增
        uint8_t left_magnet_state_;   // 存储绿灯状态
        uint8_t right_magnet_state_; // 存储黄灯状态
        std::string parameter_string_;
        // /* robot ros parameter server */
        std::map<string, float> robot_params;
        std::map<string, string> robot_devices;


        rclcpp::Service<bimax_msgs::srv::MopControl>::SharedPtr service_mop;
        rclcpp::Service<bimax_msgs::srv::LedControl>::SharedPtr service_led;
        rclcpp::Service<bimax_msgs::srv::MagnetControl>::SharedPtr service_magnet;
        rclcpp::Service<bimax_msgs::srv::CatcherControl>::SharedPtr service_catcher;
        // /* other private variables */
         std::string robot_mkey;
        rclcpp::Subscription<bimax_msgs::msg::RobotCommand>::SharedPtr cmd_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr jaw_sub;
        rclcpp::Publisher<bimax_msgs::msg::RobotState>::SharedPtr state_pub_;
        rclcpp::Publisher<bimax_msgs::msg::MotorError>::SharedPtr motor_error_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr jaw_pub_;
        bimax_msgs::msg::RobotCommand CommandValues;
        void led_handle_request(const std::shared_ptr<bimax_msgs::srv::LedControl::Request> request,std::shared_ptr<bimax_msgs::srv::LedControl::Response> response);
        void magnet_handle_request(const std::shared_ptr<bimax_msgs::srv::MagnetControl::Request> request,std::shared_ptr<bimax_msgs::srv::MagnetControl::Response> response);
        void catcher_handle_request( const std::shared_ptr<bimax_msgs::srv::CatcherControl::Request> request, std::shared_ptr<bimax_msgs::srv::CatcherControl::Response> response);
        void mop_handle_request(const std::shared_ptr<bimax_msgs::srv::MopControl::Request> request,std::shared_ptr<bimax_msgs::srv::MopControl::Response> response);
    };
}

#endif