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
        // 初始化服务
        service_mop = create_service<bimax_msgs::srv::MopControl>("mop_control",std::bind(&RosClass::mop_handle_request, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(get_logger(), "拖布控制服务已启动");
        service_catcher = create_service<bimax_msgs::srv::CatcherControl>("catcher_control",std::bind(&RosClass::catcher_handle_request, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(get_logger(), "吸尘控制服务已启动");
        service_led = create_service<bimax_msgs::srv::LedControl>("led_control",std::bind(&RosClass::led_handle_request, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(get_logger(), "LED控制服务已启动");
        service_magnet = create_service<bimax_msgs::srv::MagnetControl>("magnet_control",std::bind(&RosClass::magnet_handle_request, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(get_logger(), "磁铁控制服务已启动");
        // 状态发布
        state_pub_ = create_publisher<bimax_msgs::msg::RobotState>("/bimaxArmStateValues", 10);
        RCLCPP_INFO(this->get_logger(), "robot Driver(base communication) init successful!");
        return true;
    }
    void RosClass::commandCallback(const bimax_msgs::msg::RobotCommand::SharedPtr msg) 
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        command_queue_.push(*msg);
        queue_cv_.notify_one();  // 通知有新的命令到达
    }
    void RosClass::mop_handle_request(
        const std::shared_ptr<bimax_msgs::srv::MopControl::Request> request,
        std::shared_ptr<bimax_msgs::srv::MopControl::Response> response)
    {
        // 验证输入状态是否合法
        if (request->mop_motor_pwm > 1000 || request->mop_state > 1) {
            response->success = false;
            response->message = "错误：状态值必须是0、1";
            return;
        }
        mop_motor_pwm_state_ = request->mop_motor_pwm;
        mop_state_ = request->mop_state;
        response->success = true;
        response->message = "操作成功";
    }
   bool  RosClass::get_mop_motor_pwm_state(uint16_t  &mop_motor_pwm_state) { 
            mop_motor_pwm_state=mop_motor_pwm_state_;
        if(mop_motor_pwm_state != last_mop_motor_pwm_state_){
            last_mop_motor_pwm_state_ = mop_motor_pwm_state;          
            return true;
        }
            return false;
    }

    bool  RosClass::get_mop_state(uint8_t  &mop_state) { 
            mop_state=mop_state_;
        if(mop_state != last_mop_state_){
            last_mop_state_ = mop_state;          
            return true;
        }
            return false;
    }

    void RosClass::catcher_handle_request(
        const std::shared_ptr<bimax_msgs::srv::CatcherControl::Request> request,
        std::shared_ptr<bimax_msgs::srv::CatcherControl::Response> response)
    {
        // 验证输入状态是否合法
        if (request->catcher_gear > 2 || request->catcher_state > 1) {
            response->success = false;
            response->message = "错误：状态值必须是0、1";
            return;
        }
        catcher_gear_state_ = request->catcher_gear;
        catcher_state_ = request->catcher_state;
        response->success = true;
        response->message = "操作成功";
    }
    bool  RosClass::get_catcher_gear_state(uint8_t  &catcher_gear_state) { 
            catcher_gear_state=catcher_gear_state_;
        if(catcher_gear_state != last_catcher_gear_state_){
            last_catcher_gear_state_ = catcher_gear_state;          
            return true;
        }
            return false;
    }

    bool  RosClass::get_catcher_state(uint8_t  &catcher_state) { 
            catcher_state=catcher_state_;
        if(catcher_state != last_catcher_state_){
            last_catcher_state_ = catcher_state;          
            return true;
        }
            return false;
    }
    void RosClass::magnet_handle_request(
        const std::shared_ptr<bimax_msgs::srv::MagnetControl::Request> request,
        std::shared_ptr<bimax_msgs::srv::MagnetControl::Response> response)
    {
        // 验证输入状态是否合法
        if (request->left_magnet_state > 1 || request->right_magnet_state > 1) {
            response->success = false;
            response->message = "错误：状态值必须是0、1";
            return;
        }
        left_magnet_state_ = request->left_magnet_state;
        right_magnet_state_ = request->right_magnet_state;
        response->success = true;
        response->message = "操作成功";
    }
    bool  RosClass::get_left_magnet_state(uint8_t  &left_magnet_state) { 
            left_magnet_state=left_magnet_state_;
        if(left_magnet_state != last_left_magnet_state_){
            last_left_magnet_state_ = left_magnet_state;          
            return true;
        }
            return false;
    }

    bool  RosClass::get_right_magnet_state(uint8_t  &right_magnet_state) { 
            right_magnet_state=right_magnet_state_;
        if(right_magnet_state != last_right_magnet_state_){
            last_right_magnet_state_ = right_magnet_state;          
            return true;
        }
            return false;
    }

    void RosClass::led_handle_request(
        const std::shared_ptr<bimax_msgs::srv::LedControl::Request> request,
        std::shared_ptr<bimax_msgs::srv::LedControl::Response> response)
    {
        // 验证输入状态是否合法
        if (request->green_state > 2 || request->yellow_state > 2) {
            response->success = false;
            response->message = "错误：状态值必须是0、1或2";
            return;
        }
        green_state_ = request->green_state;
        yellow_state_ = request->yellow_state;
        response->success = true;
        response->message = "操作成功";
    }
    bool RosClass::get_green_state(uint8_t &current_state) { 
        current_state = green_state_;
        if(current_state != last_green_state_) {
            RCLCPP_INFO(get_logger(), "绿灯状态变更: %d -> %d", last_green_state_, current_state);
            last_green_state_ = current_state;
            return true; // 状态已变化
        }
        return false; // 状态未变化
    }

    bool RosClass::get_yellow_state(uint8_t &current_state) {
        current_state = yellow_state_;
        if(current_state != last_yellow_state_) {
            RCLCPP_INFO(get_logger(), "黄灯状态变更: %d -> %d",last_yellow_state_, current_state);
            last_yellow_state_ = current_state;
            return true;
        }
        return false;
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

};
    /**  
    *   @brief      get ros handle
    *   Parameters:
    *   @return     return the pointer of ros handle of RosClass   
    */
    // rclcpp::Node::SharedPtr RosClass::getHandle()
    // {
    //     return shared_from_this();
    // }
// namespace