cmake_minimum_required(VERSION 3.8)
project(bimax_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UserCommand.msg"
  "msg/ImuState.msg"
  "msg/ImuBase.msg"
  "msg/Quaternion.msg"
  "msg/GpsState.msg"
  "msg/ForceState.msg"
  "msg/MotorState.msg"
  "msg/MotorCommand.msg"
  "msg/RobotCommand.msg"
  "msg/RobotState.msg"
  "msg/Performance.msg"
  "msg/MotorError.msg" 
  "action/BimaxFunction.action"
  "srv/LedControl.srv"
  "srv/MagnetControl.srv" 
  "srv/CatcherControl.srv" 
  "srv/MopControl.srv"
  DEPENDENCIES
    std_msgs
    sensor_msgs
    control_msgs
    geometry_msgs
    builtin_interfaces
    action_msgs
  
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
