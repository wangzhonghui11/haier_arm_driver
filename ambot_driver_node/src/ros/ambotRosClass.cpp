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
        this->declare_parameter<std::string>("ambot_type", "ambot_W1");
        this->declare_parameter<std::vector<std::string>>("robot_subscribe_topic", std::vector<std::string>());
        this->declare_parameter<std::vector<std::string>>("robot_advertise_topic", std::vector<std::string>());
        this->declare_parameter("ambot_devices.motor_device", "/dev/ttyDefault");
        this->declare_parameter("ambot_params.motor_baud", 1000000);

         this->get_parameter("ambot_type", parameter_string_);
         RCLCPP_INFO(this->get_logger(), "Hello %s", parameter_string_.c_str());
        // 2. 读取设备参数
        std::string motor_device;
        if (!this->get_parameter("ambot_devices.motor_device", motor_device)) {
            RCLCPP_ERROR(this->get_logger(), "Missing 'ambot_devices.motor_device' in YAML!");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Motor device: %s", motor_device.c_str());

        // 3. 读取配置参数
        int motor_baud;
        this->get_parameter("ambot_params.motor_baud", motor_baud);


        // 6. 打印关键参数（调试用）
        RCLCPP_INFO(this->get_logger(), "Motor baud: %d", motor_baud);
        // // 5. get subscribe and advertise topics
        std::vector<std::string> subscribe_name = this->get_parameter("robot_subscribe_topic").as_string_array();
        std::vector<std::string> advertise_name = this->get_parameter("robot_advertise_topic").as_string_array();
        RCLCPP_INFO(this->get_logger(), "Subscribe topics: %zu", subscribe_name.size());
        RCLCPP_INFO(this->get_logger(), "Advertise topics: %zu", advertise_name.size());
        // // 6. get constant parameters
        // std::string constName = "ambot_const_params";
        // std::vector<float> constTempMode = this->get_parameter(constName + "/motor_control_mode").as_double_array();
        // std::vector<float> constTempMaxP = this->get_parameter(constName + "/motor_max_angle").as_double_array();
        // std::vector<float> constTempMinP = this->get_parameter(constName + "/motor_min_angle").as_double_array();
        // std::vector<float> constTempVel = this->get_parameter(constName + "/motor_velocity").as_double_array();
        // std::vector<float> constTempOffset = this->get_parameter(constName + "/motor_position_offset").as_double_array();
        // std::vector<float> constMotorAxisDir = this->get_parameter(constName + "/motor_axis_direction").as_double_array();

        // // 7. init publishers
        // if (advertise_name.size() > 0) {
        //     sensorValuePub = this->create_publisher<ambot_msg::msg::AmbotState>(advertise_name.at(0), 10);
        // }
        if (advertise_name.size() > 0) {
            terminateValuePub = this->create_publisher<std_msgs::msg::Bool>(advertise_name.at(1), 10);
        }

        // // 8. init subscribers
        // if (subscribe_name.size() > 0) {
        //     commandValueSub = this->create_subscription<ambot_msg::msg::AmbotCommand>(
        //         subscribe_name.at(0), 10, std::bind(&RosClass::commandValueCallback, this, std::placeholders::_1));
        // }
        // if (subscribe_name.size() > 1) {
        //     terminateValueSub = this->create_subscription<std_msgs::msg::Bool>(
        //         subscribe_name.at(1), 10, std::bind(&RosClass::terminateValueCallback, this, std::placeholders::_1));
        // }
        // if (subscribe_name.size() > 2) {
        //     wheelCmdValueSub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        //         subscribe_name.at(2), 10, std::bind(&RosClass::wheelCmdValueCallback, this, std::placeholders::_1));
        // }

        // // 9. init robot features
        // robotFeatures.robotType = name_type;
        // robotFeatures.motorDevName = robot_devices[robot_mkey + string("_devices/motor_device")];
        // robotFeatures.sensorDevName = robot_devices[robot_mkey + string("_devices/sensor_device")];
        // robotFeatures.motorDevBaud = (unsigned int)(robot_params[robot_mkey + string("_params/motor_baud")]);
        // robotFeatures.sensorDevBaud = (unsigned int)(robot_params[robot_mkey + string("_params/sensor_baud")]);
        // robotFeatures.jointMotorNum = (uint8_t)(robot_params[robot_mkey + string("_params/joint_motor_num")]);
        // robotFeatures.wheelMotorNum = (uint8_t)(robot_params[robot_mkey + string("_params/wheel_motor_num")]);
        // robotFeatures.motorNum = (uint8_t)(robot_params[robot_mkey + string("_params/motor_num")]);

        // // 10. resize and fill arrays
        // robotFeatures.motorControlMode.resize(robotFeatures.motorNum);
        // robotFeatures.motorMaxAngle.resize(robotFeatures.motorNum);
        // robotFeatures.motorMinAngle.resize(robotFeatures.motorNum);
        // robotFeatures.motorVelocity.resize(robotFeatures.motorNum);
        // robotFeatures.motorOffset.resize(robotFeatures.motorNum);
        // robotFeatures.motorAxisDirection.resize(robotFeatures.motorNum);

        // for (int i = 0; i < robotFeatures.motorNum; i++) {
        //     robotFeatures.motorControlMode.at(i) = (uint8_t)constTempMode.at(i);
        //     robotFeatures.motorMaxAngle.at(i) = (float)constTempMaxP.at(i);
        //     robotFeatures.motorMinAngle.at(i) = (float)constTempMinP.at(i);
        //     robotFeatures.motorVelocity.at(i) = (float)constTempVel.at(i);
        //     constTempOffset.at(i) = constTempOffset.at(i) * 3.1415926 / 180;
        //     robotFeatures.motorOffset.at(i) = (float)constTempOffset.at(i);
        //     robotFeatures.motorAxisDirection.at(i) = (float)constMotorAxisDir.at(i);
        // }

        // robotFeatures.rosRate = (uint16_t)(robot_params[robot_mkey + string("_params/ros_rate")]);
        // robotFeatures.carMaxVel = (float)(robot_params[robot_mkey + string("_params/car_max_vel")]);
        // robotFeatures.wheelRadius = (float)(robot_params[robot_mkey + string("_params/wheel_radius")]);

        // // 11. init variables
        // commandValues.command.resize(robotFeatures.motorNum);
        // ambotState.motor_state.resize(robotFeatures.motorNum);
        // wheelCmd.resize(robotFeatures.wheelMotorNum);

        RCLCPP_INFO(this->get_logger(), "robot Driver(base communication) init successful!");
        return true;
    }

    // /**  
    // *   @brief      get all parameters from param server and save data
    // *   Parameters:
    // *   @param      robot_params    [out]the map data include key and data
    // *   @param      robot_devices   [out]the map data include key and data
    // *   @return     none
    // */
    void RosClass::getParameters(map<string, float>& robot_params, map<string, string>& robot_devices)
    {
        // Get parameters using parameter client
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this);
        while (!param_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for parameter service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for parameter service...");
        }

        // Get robot parameters
        auto params = param_client->get_parameters({robot_mkey + "_params"});
        for (const auto & param : params) {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                robot_params[param.get_name()] = static_cast<float>(param.as_double());
            } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                robot_params[param.get_name()] = static_cast<float>(param.as_int());
            }
        }

        // Get robot devices
        auto device_params = param_client->get_parameters({robot_mkey + "_devices"});
        for (const auto & param : device_params) {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                robot_devices[param.get_name()] = param.as_string();
            }
        }

        // Update class members
        this->robot_params.clear();
        this->robot_devices.clear();
        this->robot_params.insert(robot_params.begin(), robot_params.end());
        this->robot_devices.insert(robot_devices.begin(), robot_devices.end());
    }

    // /**  
    // *   @brief      get single server params
    // *   Parameters:
    // *   @param      paramName   [in]the single server param name
    // *   @param      getData     [out]the data need to get
    // *   @return     none
    // */
    // bool RosClass::getSingleServerParameter(std::string paramName, int& getData)
    // {
    //     try {
    //         getData = this->get_parameter(paramName).as_int();
    //         return true;
    //     } catch (const rclcpp::ParameterTypeException& e) {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to get parameter %s: %s", paramName.c_str(), e.what());
    //         return false;
    //     }
    // }

    // /**  
    // *   @brief      set single server params
    // *   Parameters:
    // *   @param      robot_params    [out]the single server param name
    // *   @param      setData         [out]the data need to set
    // *   @return     none
    // */
    // template <typename T>
    // void RosClass::setSingleServerParameter(string robot_params, T setData)
    // {
    //     this->set_parameter(rclcpp::Parameter(robot_params, setData));
    // }

    // /**  
    // *   @brief      set the terminate value indicate node is alive
    // *   Parameters:
    // *   @return     none
    // */
    // void RosClass::setTerminateValue()
    // {
    //     auto msg = std_msgs::msg::Bool();
    //     msg.data = true;
    //     terminateValuePub->publish(msg);
    // }

    // /**  
    // *   @brief      output the motor data to argv
    // *   Parameters:
    // *   @param      data    [in]receive the motor data
    // *   @return     none
    // */
    // void RosClass::getJointMotorCommand(ambot_msg::msg::AmbotCommand& data)
    // {
    //     assert(commandValues.command.size() >= robotFeatures.motorNum);
    //     data = commandValues;
    //     for (int i = 0; i < robotFeatures.motorNum; i++) {
    //         data.command[i].q -= robotFeatures.motorOffset.at(i);
    //         data.command[i].q *= robotFeatures.motorAxisDirection.at(i);
    //         data.command[i].dq *= robotFeatures.motorAxisDirection.at(i);
    //         data.command[i].tor *= robotFeatures.motorAxisDirection.at(i);
    //     }
    // }

    // /**  
    // *   @brief      the callback of receive motor data function
    // *   Parameters:
    // *   @param      array    [in]the data receive from the locomotion controller for each joint 
    // *   @return     none
    // */
    // void RosClass::wheelCmdValueCallback(const std_msgs::msg::Float32MultiArray::SharedPtr array)
    // {
    //     assert(array->data.size() == robotFeatures.wheelMotorNum);
    //     wheelCmd = array->data;
    // }

    // /**  
    // *   @brief      the callback of receive motor data function
    // *   Parameters:
    // *   @param      array    [in]the data receive from the locomotion controller for each joint 
    // *   @return     none
    // */
    // void RosClass::commandValueCallback(const ambot_msg::msg::AmbotCommand::SharedPtr array)
    // {
    //     assert(array->command.size() <= robotFeatures.motorNum);
    //     commandValues = *array;
    // }

    // /**  
    // *   @brief      the callback of terminate function
    // *   Parameters:
    // *   @param      termNode    [in]the data could judge whether need to close current ros node
    // *   @return     none
    // */
    // void RosClass::terminateValueCallback(const std_msgs::msg::Bool::SharedPtr termNode)
    // {
    //     terminate = termNode->data;
    // }

    // /**  
    // *   @brief      update motor value
    // *   Parameters:
    // *   @return     none
    // */
    // void RosClass::getWheelMotorCommand(std::vector<float> &out)
    // {
    //     out = wheelCmd;
    //     for (int i = 0; i < robotFeatures.wheelMotorNum; i++) {
    //         out.at(i) *= robotFeatures.motorAxisDirection.at(i + robotFeatures.jointMotorNum);
    //     }
    // }

    // void RosClass::robotFbValuePub(const ambot_msg::msg::AmbotState& data)
    // {
    //     ambot_msg::msg::AmbotState values;
    //     assert(data.motor_state.size() <= robotFeatures.motorNum);
    //     values = data;
    //     for (int i = 0; i < robotFeatures.motorNum; i++) {
    //         values.motor_state[i].pos *= robotFeatures.motorAxisDirection.at(i);
    //         values.motor_state[i].vel *= robotFeatures.motorAxisDirection.at(i);
    //         values.motor_state[i].cur *= robotFeatures.motorAxisDirection.at(i);
    //         values.motor_state.at(i).pos += robotFeatures.motorOffset.at(i);
    //     }
    //     sensorValuePub->publish(values);
    // }

    // /**  
    // *   @brief      check current program update frequency and judge whether output warning message
    // *   Parameters:
    // *   @return     none 
    // */
    // void RosClass::topicFrequencyWarning(void)
    // {
    //     // 1. get current time and init variables
    //     static long int deltaTime = 0;
    //     static bool firstFlag = true;
    //     static auto time_old = std::chrono::high_resolution_clock::now();
    //     auto time_new = std::chrono::high_resolution_clock::now();
        
    //     // 2. get the internal time between two update
    //     deltaTime = std::chrono::duration_cast<std::chrono::microseconds>(time_new - time_old).count();
        
    //     // 3. check frequency
    //     if ((deltaTime > 3 * 1000000 / robotFeatures.rosRate) && !firstFlag) {
    //         RCLCPP_WARN(this->get_logger(), 
    //             "the frequency of Ros topic publish is too slow, frequency: %.3f", 
    //             (double)(1000000 / deltaTime));
    //     }            
        
    //     time_old = time_new;
    //     if (firstFlag) {
    //         firstFlag = false;
    //     }
    // }

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