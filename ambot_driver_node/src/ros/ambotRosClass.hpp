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
#include "ambotDriver.hpp"
#include <cassert>
// #include "privateProtocol.hpp"

namespace ambot_driver_ns
{
    using namespace std;

    class RosClass : public rclcpp::Node
    {
    public:
        /* terminate flag */
        bool terminate = false;

        /* all ros parameters server:robot params */
        RobotDriver_TP robotFeatures;

        /* construct and deconstruct function */
        RosClass(int argc, char** argv, const std::string& node_name = "yangtze_node");
        ~RosClass();

        /* open function api */
        bool init();
        void rosSleep();

        /* open api for output command */
        // void getWheelMotorCommand(std::vector<float> &out);
        bool getJointMotorCommand(bimax_msgs::msg::RobotCommand& msg);
        // void robotFbValuePub(const ambot_msg::msg::AmbotState& data);
        
        /* other open function */
        // void setTerminateValue();
        void getParameters(map<string, float>& robot_params, map<string, string>& robot_devices);
        // template <typename T> void setSingleServerParameter(string robot_params, T setData);
        // bool getSingleServerParameter(std::string paramName, int& getData);
        // rclcpp::Node::SharedPtr getHandle();
        // void topicFrequencyWarning(void);
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
    private:
        /* public topic */
        // rclcpp::Publisher<ambot_msg::msg::AmbotState>::SharedPtr sensorValuePub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr terminateValuePub;
        std::queue<bimax_msgs::msg::RobotCommand> command_queue_;
        std::mutex queue_mutex_;
        std::condition_variable queue_cv_;
        // /* subscribe topic */
        // rclcpp::Subscription<ambot_msg::msg::AmbotCommand>::SharedPtr commandValueSub;
        // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheelCmdValueSub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr terminateValueSub;

        // /* tf broadcaster */
        // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // /* receive command buffer */
        // ambot_msg::msg::AmbotCommand commandValues;
        // std::vector<float> wheelCmd;
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
        bimax_msgs::msg::RobotCommand CommandValues;
        void led_handle_request(const std::shared_ptr<bimax_msgs::srv::LedControl::Request> request,std::shared_ptr<bimax_msgs::srv::LedControl::Response> response);
        void magnet_handle_request(const std::shared_ptr<bimax_msgs::srv::MagnetControl::Request> request,std::shared_ptr<bimax_msgs::srv::MagnetControl::Response> response);
        void catcher_handle_request( const std::shared_ptr<bimax_msgs::srv::CatcherControl::Request> request, std::shared_ptr<bimax_msgs::srv::CatcherControl::Response> response);
        void mop_handle_request(const std::shared_ptr<bimax_msgs::srv::MopControl::Request> request,std::shared_ptr<bimax_msgs::srv::MopControl::Response> response);
    };
}

#endif