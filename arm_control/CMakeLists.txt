cmake_minimum_required(VERSION 3.8)
project(arm_control)

# 寻找依赖
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rosidl_typesupport_c REQUIRED)  # 新增
find_package(rosidl_typesupport_cpp REQUIRED)  # 新增

# find_package(pymodbus REQUIRED)
find_package(omni_msgs REQUIRED)  # 新增

# 声明消息文件
set(MSG_FILES "msg/ControlCommand.msg")
set(SRV_FILES "srv/RobotArmCommand.srv")  # 新增服务文件

# set(SRV_FILES "srv/GripperControl.srv")  # 新增服务文件
# set(SRV_FILES "srv/SetGripperParam.srv")  # 新增服务文件
# set(SRV_FILES "srv/SetGripperPosition.srv")  # 新增服务文件

# 生成消息接口（核心步骤）
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}  # 添加服务文件
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
  # ROSIDL_TYPESUPPORT_DEPENDENCIES rosidl_typesupport_c;rosidl_typesupport_cpp
)

install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_c/
  DESTINATION lib
)
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_cpp/
  DESTINATION lib
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# 安装Python源文件（关键）
install(
  DIRECTORY arm_control/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

# 安装生成的消息文件（如果需要自定义消息）
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}/msg/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}/msg/
)

install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}/srv/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}/srv/
)

# 安装配置文件目录（新增）
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装可执行节点
install(PROGRAMS
  arm_control/robot_arm_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME robot_arm_node
)

# 安装可执行节点
install(PROGRAMS
  arm_control/gripper.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gripper
)

# 安装 Python 类型支持绑定文件（关键）
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.so"  # 匹配生成的 C 扩展文件
)

# 导出依赖（完整）
ament_export_dependencies(
  rosidl_default_runtime
  # rosidl_typesupport_c  # 新增
  # rosidl_typesupport_cpp  # 新增
)

ament_package()