#include "ambotRosClass.hpp"
#include <sys/time.h>
#include <chrono>

namespace ambot_driver_ns
{
    /**  
    *   @brief      construct function of RosClass
    *   Parameters:
    *   @return     none
    */
    RosClass::RosClass(int argc, char** argv, const std::string& node_name)
        : Node(node_name)
    {
        // 1. init variables
        robot_mkey = string("ambot");

        // 2. check if ROS2 is initialized properly
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "ROS2 not initialized properly!");
            rclcpp::shutdown();
        }
        this->declare_parameter("init_flag", true); 
        
        // 3. init RosClass class
        if (!init()) {
            RCLCPP_ERROR(this->get_logger(), "RosClass initialization failed!");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "Robot RosClass Interface init successfully!");
    }

    /**  
    *   @brief      deconstruct function of RosClass
    *   Parameters:
    *   @return     none
    */
    RosClass::~RosClass()
    {
        RCLCPP_INFO(this->get_logger(), "robot driver node just terminated!");
        
        // Clear all resources
        // robot_params.clear();
        // robot_devices.clear();
    }

    /**  
    *   @brief      ros sleep function according to ros rate
    *   Parameters:
    *   @return     none
    */
    void RosClass::rosSleep()
    {
        rclcpp::sleep_for(std::chrono::milliseconds(1000 / 100));
    }
    
    /**  
    *   @brief      init api of RosClass
    *   Parameters:
    *   @return     true:init successful; false:init failed    
    */
    bool RosClass::init()
    {
        rclcpp::QoS command_qos(1);
        command_qos.best_effort();

        this->declare_parameter<std::string>("ambot_type", "ambot_W1");
        this->declare_parameter<std::vector<std::string>>("robot_subscribe_topic", std::vector<std::string>());
        this->declare_parameter<std::vector<std::string>>("robot_advertise_topic", std::vector<std::string>());
        this->declare_parameter("ambot_devices.motor_device", "/dev/ttyDefault");
        this->declare_parameter("ambot_params.motor_baud", 1000000);
        this->get_parameter("ambot_type", parameter_string_);
        // 2. 读取设备参数
        std::string motor_device;
        if (!this->get_parameter("ambot_devices.motor_device", motor_device)) {
            RCLCPP_ERROR(this->get_logger(), "Missing 'ambot_devices.motor_device' in YAML!");
            return false;
        }

        // 3. 读取配置参数
        int motor_baud;
        this->get_parameter("ambot_params.motor_baud", motor_baud);
        // // 5. get subscribe and advertise topics
        std::vector<std::string> subscribe_name = this->get_parameter("robot_subscribe_topic").as_string_array();
        std::vector<std::string> advertise_name = this->get_parameter("robot_advertise_topic").as_string_array();
        // // 6. get constant parameters
        if (advertise_name.size() > 0) {
            terminateValuePub = this->create_publisher<std_msgs::msg::Bool>(advertise_name.at(1), 10);
        }
        cmd_sub_ = create_subscription<bimax_msgs::msg::RobotCommand>("/bimaxArmCommandValues", command_qos,
        std::bind(&RosClass::commandCallback, this, std::placeholders::_1));

    // 状态发布
        state_pub_ = create_publisher<bimax_msgs::msg::RobotState>("/bimaxArmStateValues", 10);

        // // 8. init subscribe     
        RCLCPP_INFO(this->get_logger(), "robot Driver(base communication) init successful!");
        return true;
    }
    void RosClass::commandCallback(const bimax_msgs::msg::RobotCommand::SharedPtr msg) 
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        command_queue_.push(*msg);
        queue_cv_.notify_one();  // 通知有新的命令到达
    }

    // /**  
    // *   @brief      output the motor data to argv
    // *   Parameters:
    // *   @param      data    [in]receive the motor data
    // *   @return     none
    // */
    bool RosClass::getJointMotorCommand(bimax_msgs::msg::RobotCommand& cmd) {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    
    if (queue_cv_.wait_for(lock, std::chrono::milliseconds(10), 
                         [this]{ return !command_queue_.empty(); }))
    {
        cmd = command_queue_.front();
        command_queue_.pop();
        return true;
    }
    return false;
    }

 
    // /**  
    // *   @brief      get ros handle
    // *   Parameters:
    // *   @return     return the pointer of ros handle of RosClass   
    // */
    // rclcpp::Node::SharedPtr RosClass::getHandle()
    // {
    //     return shared_from_this();
    // }
} // namespace