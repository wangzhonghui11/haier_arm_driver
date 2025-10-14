cmake_minimum_required(VERSION 3.8)
project(ambot_driver_node)

##############################################################################
# 查找依赖
##############################################################################
# 启用新版头文件安装行为（兼容所有ROS2版本）
option(USE_SCOPED_HEADER_INSTALL_DIR "Use scoped header install directory" ON)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 设置编译选项
set(show_read_feedback_period disable)
set(show_debug_message enable)

# 添加全局宏定义
if("${show_debug_message}" STREQUAL "enable")
  add_compile_definitions(AMBOT_DEBUG)
  message("\nshow all debug messages!!!\n")
endif()

if("${show_read_feedback_period}" STREQUAL "enable")
  add_compile_definitions(SHOW_READ_FEEDBACK_PERIOD)
  message("attention, controller will use tensorboard_logger to save data!!!!!")
endif()

##############################################################################
# 构建配置
##############################################################################

# 设置源文件目录
include_directories(
  include
  src
  src/driver
  src/ros
  src/protocol
)

# 自动添加库和可执行文件
ament_auto_add_library(ambot_lib STATIC
  src/driver/ambotDriver.cpp
  src/driver/robot.cpp
  # src/protocol/privateProtocol.cpp
  # src/protocol/crcModbus.cpp
  src/ros/ambotRosClass.cpp
)

ament_auto_add_executable(ambot_node
  main.cpp
)

# 链接库
target_link_libraries(ambot_node
  ambot_lib
)

##############################################################################
# 安装配置
##############################################################################

# 自动安装所有必要文件
ament_auto_package(
  INSTALL_TO_SHARE

  launch
  config
  USE_SCOPED_HEADER_INSTALL_DIR
)

# 如果需要特殊处理某些文件的安装，可以额外添加
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)