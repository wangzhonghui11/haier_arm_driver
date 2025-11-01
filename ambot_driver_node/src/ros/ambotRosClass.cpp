#include "ambotRosClass.hpp"
#include <sys/time.h>
#include <chrono>

namespace bimax_driver_ns
{
    /**  
    *   @brief      construct function of RosClass
    *   Parameters:
    *   @return     none
    */
   bimax_msgs::msg::MotorState createMotorState(int id = 0, float q = 0.0f, float dq = 0.0f, float ddq = 0.0f, float tau = 0.0f )
    {
        bimax_msgs::msg::MotorState m;
        // m.id = id;
        m.q = q;
        m.dq = dq;
        m.ddq = ddq;
        m.tau = tau;
        return m;
    }
    RosClass::RosClass(int argc, char** argv, const std::string& node_name)
        : Node(node_name)
    {
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
        
    }

    /**  
    *   @brief      ros sleep function according to ros rate
    *   Parameters:
    *   @return     none
    */
    void RosClass::rosSleep()
    {
        rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
    
    /**  
    *   @brief      init api of RosClass
    *   Parameters:
    *   @return     true:init successful; false:init failed    
    */
    bool RosClass::init()
    {
        rclcpp::QoS command_qos(1);  // 队列深度=1
        command_qos.best_effort();   // 最大努力，低延迟
        command_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE); // 不保存历史
        this->declare_parameter("bimax_server.mop_service_name", "mop_control");
        this->declare_parameter("bimax_server.catcher_service_name", "catcher_control");
        this->declare_parameter("bimax_server.led_service_name", "led_control");
        this->declare_parameter("bimax_server.magnet_service_name", "magnet_control");
        this->declare_parameter<std::string>("bimax_type", "bimax_W1");
        this->declare_parameter("bimax_devices.motor_device", "/dev/ttyDefault");
        this->declare_parameter("bimax_params.motor_baud", 1000000);
        this->get_parameter("bimax_type", parameter_string_);
        this->declare_parameter("bimax_topic.motor_command_topic", "/bimaxArmCommandValues");
        this->declare_parameter("bimax_topic.gripper_position_topic", "/gripper_position");
        this->declare_parameter("bimax_topic.motor_states_topic", "/bimaxArmStateValues");
        this->declare_parameter("bimax_topic.motor_error_topic", "/bimax_motor_error");
        // 2. 读取设备参数
        if (!this->get_parameter("bimax_devices.motor_device", motor_device)) {
            RCLCPP_ERROR(this->get_logger(), "Missing 'bimax_devices.motor_device' in YAML!");
            return false;
        }
        this->get_parameter("bimax_params.motor_baud", motor_baud);

        //2. 主题发布订阅
        std::string robot_cmd_topic = this->get_parameter("bimax_topic.motor_command_topic").as_string();
        std::string gripper_topic = this->get_parameter("bimax_topic.gripper_position_topic").as_string();
        std::string robot_state_topic = this->get_parameter("bimax_topic.motor_states_topic").as_string();
        std::string motor_states_topic = this->get_parameter("bimax_topic.motor_error_topic").as_string();
        cmd_sub_ = create_subscription<bimax_msgs::msg::RobotCommand>(robot_cmd_topic, command_qos,
        std::bind(&RosClass::commandCallback, this, std::placeholders::_1));
        jaw_sub = create_subscription<std_msgs::msg::Float32>(gripper_topic,command_qos,std::bind(&RosClass::jawCallback, this, std::placeholders::_1));
  
        state_pub_ = create_publisher<bimax_msgs::msg::RobotState>(robot_state_topic, 10);
        motor_error_pub_ = create_publisher<bimax_msgs::msg::MotorError>(motor_states_topic, 10);
        jaw_pub_ = create_publisher<std_msgs::msg::Float32>("/gripper_pos_states", 10);
        // 初始化服务
        std::string mop_name = this->get_parameter("bimax_server.mop_service_name").as_string();
        std::string catcher_name = this->get_parameter("bimax_server.catcher_service_name").as_string();
        std::string led_name = this->get_parameter("bimax_server.led_service_name").as_string();
        std::string magnet_name = this->get_parameter("bimax_server.magnet_service_name").as_string();
        
        service_mop = create_service<bimax_msgs::srv::MopControl>(
            mop_name, std::bind(&RosClass::mop_handle_request, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(get_logger(), "Mop control service '%s' init successful!", mop_name.c_str());
        
        service_catcher = create_service<bimax_msgs::srv::CatcherControl>(
            catcher_name, std::bind(&RosClass::catcher_handle_request, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(get_logger(), "Vacuum control service '%s' init successful!", catcher_name.c_str());
        
        service_led = create_service<bimax_msgs::srv::LedControl>(
            led_name, std::bind(&RosClass::led_handle_request, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(get_logger(), "LED control service '%s' init successful!", led_name.c_str());
        
        service_magnet = create_service<bimax_msgs::srv::MagnetControl>(
            magnet_name, std::bind(&RosClass::magnet_handle_request, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(get_logger(), "Magnet control service '%s' init successful!", magnet_name.c_str());
        
        RCLCPP_INFO(this->get_logger(), "robot parameter init successful!");
        return true;

    }
    void RosClass::jawCallback(const std_msgs::msg::Float32::SharedPtr msg) 
    {
        jaw_cmd_=msg->data;
    }
    bool  RosClass::get_jaw_cmd(float  &jaw_cmd) { 
        jaw_cmd=jaw_cmd_;
        if(jaw_cmd != last_jaw_cmd_){
            last_jaw_cmd_ = jaw_cmd;          
            return true;
        }
            return false;
    }
    void RosClass::commandCallback(const bimax_msgs::msg::RobotCommand::SharedPtr msg) 
    {
        // 如果处理逻辑简单，直接处理避免队列
        latest_command_ = *msg;  // 原子操作或简单的内存拷贝
        command_ready_ = true;
    }
    void RosClass::mop_handle_request(
        const std::shared_ptr<bimax_msgs::srv::MopControl::Request> request,
        std::shared_ptr<bimax_msgs::srv::MopControl::Response> response)
    {
        // 验证输入状态是否合法
        if (request->mop_motor_pwm > 2000 || request->mop_state > 1) {
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
    
        if (command_ready_) {
            cmd = latest_command_;
            command_ready_ = false;
            return true;
        }
        return false;
    }

    void RosClass::robotFbValuePub(YiyouMecArm &mecarm,float &lef ,float &righ,float & jaw_pos)
    {
    auto state_msg = bimax_msgs::msg::RobotState();
    state_msg.motor_state.push_back(createMotorState(0, lef, 0, 0.0f, 0)); 
    state_msg.motor_state.push_back(createMotorState(1, mecarm.pos_rad_motor1, mecarm.vel_rad_s_motor1, 0.0f, mecarm.torque_nm_motor1));
    state_msg.motor_state.push_back(createMotorState(2, mecarm.pos_rad_motor2, mecarm.vel_rad_s_motor2, 0.0f, mecarm.torque_nm_motor2));  
    state_msg.motor_state.push_back(createMotorState(3, 0, 0, 0.0f, 0)); 
    state_msg.motor_state.push_back(createMotorState(4, righ, 0, 0.0f, 0)); 
    state_msg.motor_state.push_back(createMotorState(5, mecarm.pos_rad_motor3, mecarm.vel_rad_s_motor3, 0.0f, mecarm.torque_nm_motor3));  
    state_msg.motor_state.push_back(createMotorState(6, mecarm.pos_rad_motor4, mecarm.vel_rad_s_motor4, 0.0f, mecarm.torque_nm_motor4));
    state_msg.motor_state.push_back(createMotorState(7, mecarm.pos_rad_motor5, mecarm.vel_rad_s_motor5, 0.0f, mecarm.torque_nm_motor5)); 
    auto error_msg = bimax_msgs::msg::MotorError();
    error_msg.state_id1 = mecarm.status_motor1;
    error_msg.state_id2 = mecarm.status_motor2;  // 假设有 status_motor2
    error_msg.state_id3 = 0XFF;  // 假设有 status_motor3
    error_msg.state_id4 = mecarm.status_motor3;  // 假设有 status_motor4
    error_msg.state_id5 = mecarm.status_motor4;  // 假设有 status_motor5
    error_msg.state_id6 = mecarm.status_motor5;  // 假设有 status_motor6
    error_msg.error_id1 = mecarm.error_motor1;
    error_msg.error_id2 = mecarm.error_motor2;
    error_msg.error_id3 = 0;
    error_msg.error_id4 = mecarm.error_motor3;
    error_msg.error_id5 = mecarm.error_motor4;
    error_msg.error_id6 = mecarm.error_motor5;
    auto jaw_angle_msg = std_msgs::msg::Float32();
    jaw_angle_msg.data=jaw_pos;
    state_pub_->publish(state_msg);
    motor_error_pub_->publish(error_msg);
    jaw_pub_->publish(jaw_angle_msg);
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