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
// #include "nav_msgs/msg/odometry.hpp"
// #include "tf2_ros/transform_broadcaster.h"
// #include "ambot_msg/msg/ambot_state.hpp"
// #include "ambot_msg/msg/ambot_command.hpp"
// #include "sensor_msgs/msg/imu.hpp"
// #include "ambot_msg/msg/joint_state.hpp"
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
        // void getJointMotorCommand(ambot_msg::msg::AmbotCommand& data);
        // void robotFbValuePub(const ambot_msg::msg::AmbotState& data);
        
        /* other open function */
        // void setTerminateValue();
        void getParameters(map<string, float>& robot_params, map<string, string>& robot_devices);
        // template <typename T> void setSingleServerParameter(string robot_params, T setData);
        // bool getSingleServerParameter(std::string paramName, int& getData);
        // rclcpp::Node::SharedPtr getHandle();
        // void topicFrequencyWarning(void);

    private:
        /* public topic */
        // rclcpp::Publisher<ambot_msg::msg::AmbotState>::SharedPtr sensorValuePub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr terminateValuePub;

        // /* subscribe topic */
        // rclcpp::Subscription<ambot_msg::msg::AmbotCommand>::SharedPtr commandValueSub;
        // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheelCmdValueSub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr terminateValueSub;

        // /* tf broadcaster */
        // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // /* receive command buffer */
        // ambot_msg::msg::AmbotCommand commandValues;
        // std::vector<float> wheelCmd;

        // /* sensor and motor feedback data buffer */
        // ambot_msg::msg::AmbotState ambotState;
        std::string parameter_string_;
        // /* robot ros parameter server */
        std::map<string, float> robot_params;
        std::map<string, string> robot_devices;

        // /* other private variables */
         std::string robot_mkey;

        // /* timer for periodic operations */
        // rclcpp::TimerBase::SharedPtr timer_;

        // /* subscribe topic callback function */
        // void commandValueCallback(const ambot_msg::msg::AmbotCommand::SharedPtr array);
        // void wheelCmdValueCallback(const std_msgs::msg::Float32MultiArray::SharedPtr array);
        // void terminateValueCallback(const std_msgs::msg::Bool::SharedPtr termNode);
    };
}

#endif