#!/usr/bin/env python3
import sys
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from omni_msgs.msg import OmniState
from omni_msgs.msg import OmniButtonEvent
from scipy.spatial.transform import Rotation as R

import time
import numpy as np

sys.path.insert(0, '/home/lz/DAMR/install/arm_control/lib/python3.10/site-packages')

# 导入自定义模块
from arm_control.robotarm_sdk import RobotArmSDK
from arm_control.msg import ControlCommand  # 自定义消息类型
from arm_control.srv import RobotArmCommand  # 确保自定义服务已正确定义

class RobotArmController(Node):
    """机械臂控制节点，支持单一话题接收双模式控制指令，并提供服务接口"""
    
    def __init__(self):
        self.is_command = 0
        # 第一步：从命令行参数中提取节点名
        node_name = "robot_arm_controller"  # 默认节点名
        
        # 手动解析命令行参数中的 node_name
        for arg in sys.argv:
            if arg.startswith('robot_name:='):
                node_name = arg.split(':=')[1]
                break

        # 第二步：调用父类 Node 的构造函数
        super().__init__(node_name)
        
        # 声明参数
        self.declare_parameter('robot_name', 'arm_left')
        self.declare_parameter('robot_ip', '192.168.1.13')
        self.declare_parameter('robot_port', 6001)
        self.declare_parameter('control_mode', 'joint')  # 默认关节控制模式
        self.declare_parameter('omni_command_topic', '/phantom/state')  # 默认关节控制模式
        self.declare_parameter('omni_button_event_topic', '/phantom/button')  # 默认关节控制模式

        
        # 获取参数值
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.robot_port = self.get_parameter('robot_port').get_parameter_value().integer_value
        self.control_mode = self.get_parameter('control_mode').get_parameter_value().string_value
        self.omni_command_topic = self.get_parameter('omni_command_topic').get_parameter_value().string_value
        self.omni_button_topic = self.get_parameter('omni_button_event_topic').get_parameter_value().string_value

        
        # 固定运动速度
        self.MOVEMENT_SPEED = 60  # 固定速度50%
        
        # 初始化机械臂SDK
        self.robot_arm = RobotArmSDK(self.robot_ip, self.robot_port)
        self.connected = False
        
        # 控制模式映射
        self.mode_map = {
            'joint': 0,    # 关节坐标
            'cartesian': 1 # 直角坐标
        }
        
        # 构建话题名称
        self.command_topic = f'/{self.robot_name}/control_command'
        self.joint_state_topic = f'/{self.robot_name}/joint_states'
        self.cartesian_state_topic = f'/{self.robot_name}/cartesian_states'
        self.status_topic = f'/{self.robot_name}/robot_status'
        self.mode_topic = f'/{self.robot_name}/control_mode'
        self.fault_topic = f'/{self.robot_name}/fault_status'
        self.joint_command_topic = f'/{self.robot_name}/joint_command'
        # self.omni_command_topic = f'/{self.robot_name}/omni_command'
        # self.omni_command_topic = f'/phantom/state'
        
        # 构建服务名称
        self.control_service_name = f'/{self.robot_name}/control_service'
        
        # 连接机械臂
        self.connect_to_robot()
        
        # ROS 订阅者和发布者
        self.command_sub = self.create_subscription(
            ControlCommand,
            self.command_topic,
            self.command_callback,
            10)
        
        self.joint_state_pub = self.create_publisher(
            JointState,
            self.joint_state_topic,
            10)
        
        self.cartesian_state_pub = self.create_publisher(
            Float64MultiArray,
            self.cartesian_state_topic,
            10)
        
        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            10)
        
        self.mode_pub = self.create_publisher(
            String,
            self.mode_topic,
            10)
        
        # 添加故障状态发布者
        self.fault_pub = self.create_publisher(
            String,
            self.fault_topic,
            10)
        
        # 创建服务（使用自定义服务类型）
        self.control_service = self.create_service(
            RobotArmCommand,
            self.control_service_name,
            self.control_service_callback)
        
        self.joint_command_sub = self.create_subscription(
            JointState,
            self.joint_command_topic,
            self.joint_command_callback,
            10)
        
        self.omni_command_sub = self.create_subscription(
            OmniState,
            self.omni_command_topic,
            self.omni_command_callback,
            10)
        
        self.omni_command_sub = self.create_subscription(
            OmniButtonEvent,
            self.omni_button_topic,
            self.omni_button_callback,
            10)
        
        # 参数动态更新回调
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # 状态更新定时器
        self.status_timer = self.create_timer(0.1, self.update_robot_status)
        
        self.get_logger().info(f'{self.robot_name} 控制节点已启动')
        self.get_logger().info(f'当前控制模式: {self.control_mode}，固定速度: {self.MOVEMENT_SPEED}%')
        self.get_logger().info(f'控制服务已启动: {self.control_service_name}')
        self.get_logger().info(f'关节指令话题已订阅: {self.joint_command_topic}')  # 新增日志
    
    def connect_to_robot(self):
        """连接到机械臂并进行初始化"""
        self.get_logger().info(f'{self.robot_name} 尝试连接到: {self.robot_ip}:{self.robot_port}')
        
        if self.robot_arm.connect():
            self.connected = True
            self.get_logger().info(f'{self.robot_name} 连接成功')
            
            # # 初始化机械臂状态
            # time.sleep(1)
            # self.robot_arm.fault_reset(1)
            # time.sleep(0.5)
            
            # if self.robot_arm.servo_status_inquire(1) == 0:
            #     # 设置伺服状态为就绪
            #     self.robot_arm.servo_status_set(1, 1)
            #     time.sleep(0.5)

            # if self.robot_arm.deadman_status_inquire() != 1:
            #     # 设置上电状态
            #     self.robot_arm.deadman_status_set(1)
            #     time.sleep(0.5)

            current_status = self.robot_arm.servo_status_inquire(1)
            while current_status != 3:
                self.robot_arm.operation_mode_set(0)
                self.robot_arm.fault_reset(1)
                self.robot_arm.servo_status_set(1, 1)
                self.robot_arm.deadman_status_set(0)
                self.robot_arm.deadman_status_set(1)
                self.robot_arm.servo_status_inquire(1)
                time.sleep(1)
                current_status = self.robot_arm.servo_status_inquire(1)
            
            self.get_logger().info(f'{self.robot_name} 初始化完成')
        else:
            self.get_logger().error(f'{self.robot_name} 连接失败')
            # 定时重试连接
            self.reconnect_timer = self.create_timer(5.0, self.reconnect_callback)
    
    def reconnect_callback(self):
        """定时重试连接机械臂"""
        if not self.connected:
            self.get_logger().info(f'{self.robot_name} 尝试重新连接...')
            if self.robot_arm.reconnect():
                self.connected = True
                self.get_logger().info(f'{self.robot_name} 重新连接成功')
                self.destroy_timer(self.reconnect_timer)
    
    def parameter_callback(self, params):
        """参数动态更新回调"""
        for param in params:
            if param.name == 'control_mode':
                if param.value in self.mode_map:
                    self.control_mode = param.value
                    self.get_logger().info(f'控制模式已切换为: {self.control_mode}')
                    return rclpy.Parameter.Response(successful=True)
                else:
                    self.get_logger().error(f'无效的控制模式: {param.value}')
                    return rclpy.Parameter.Response(successful=False, reason='无效的控制模式')
        
        return rclpy.Parameter.Response(successful=True)
    
    # 新增关节指令回调函数
    def joint_command_callback(self, msg):
        """处理JointState类型的关节控制指令"""
        if not self.connected:
            self.get_logger().warning(f'{self.robot_name} 未连接，无法执行关节指令')
            return
            
        # 验证关节位置数据长度（假设机械臂有7个关节）
        if len(msg.position) != 7:
            self.get_logger().error(f'{self.robot_name} 关节指令长度无效: {len(msg.position)}, 期望7个关节值')
            return
            
        # 提取关节目标位置
        target_pos = list(msg.position)
        
        # 关节控制模式固定为0
        coord_type = self.mode_map['joint']
        self.get_logger().info(f'{self.robot_name} 收到关节位置指令: {[round(p, 2) for p in target_pos]}')
        self.execute_movement(target_pos, coord_type)


    def quaternion_to_euler(self,x, y, z, w):
        """
        将四元数转换为欧拉角（ZYX顺序，即yaw-pitch-roll）
        返回值：roll, pitch, yaw（单位：弧度）
        """
        # 计算roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # 计算pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # 奇异值处理
        else:
            pitch = np.arcsin(sinp)
        
        # 计算yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def omni_button_callback(self, msg):
        print(msg)
        """处理合并后的控制指令"""
        if msg.white_button:
            self.is_command = 1
        else:
            self.is_command = 0


    def omni_command_callback(self, msg):
        """
        处理OmniState类型消息，提取位姿信息控制机械臂运动
        直角坐标模式下pos格式：[x, y, z, a, b, c, 0.0]
        其中a, b, c由四元数转换为欧拉角（单位：度）
        """
        if not self.connected:
            self.get_logger().warning(f'{self.robot_name} 未连接，无法执行Omni指令')
            return
        
        try:
            # 1. 提取位置信息（米）
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            
            # 2. 提取四元数并转换为欧拉角（单位：弧度 -> 转换为度）
            quat = [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]
            
            # # 四元数转欧拉角（roll, pitch, yaw），使用tf库的转换函数
            # roll, pitch, yaw = self.quaternion_to_euler(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w)
            
            # # 转换为角度（机械臂通常使用角度单位）
            # a = np.degrees(roll)/180 *3.14   # 绕X轴旋转
            # b = np.degrees(pitch)/180 *3.14  # 绕Y轴旋转
            # c = np.degrees(yaw)/180 *3.14    # 绕Z轴旋转
            c,b,a =R.from_quat(quat).as_euler("zyx",degrees=False) #返回rad
            
            # 3. 构造机械臂所需的目标位置列表（直角坐标模式下第七位为0）
            target_pos = [x, y, z, -a, b, -c, 0.0]
            
            # 4. 检查夹爪状态（可选：这里仅打印日志，可根据需求扩展夹爪控制）
            # if msg.close_gripper:
            #     self.get_logger().info(f"{self.robot_name} 夹爪闭合指令")
            # else:
            #     self.get_logger().info(f"{self.robot_name} 夹爪打开指令")
            
            # 5. 使用直角坐标模式执行运动（coord_type=1）
            coord_type = self.mode_map['cartesian']
            # self.get_logger().info(
            #     f"{self.robot_name} 收到Omni位姿指令: "
            #     f"位置(x={x:.3f}, y={y:.3f}, z={z:.3f}), "
            #     f"姿态(a={a:.1f}, b={b:.1f}, c={c:.1f})"
            # )
            if self.is_command:
                self.execute_movement(target_pos, coord_type)
            # else:
            #     self.get_logger().info(f"{self.robot_name} 不移动机械臂")
            
        except Exception as e:
            self.get_logger().error(f"{self.robot_name} Omni指令处理错误: {str(e)}")
    
    def command_callback(self, msg):
        """处理合并后的控制指令"""
        if not self.connected:
            self.get_logger().warning(f'{self.robot_name} 未连接，无法执行指令')
            return
            
        # 验证位数据长度
        if len(msg.position) != 7:
            self.get_logger().error(f'{self.robot_name} 无效指令长度: {len(msg.position)}, 期望7个值')
            return
            
        target_pos = list(msg.position)
        
        # 根据控制模式执行相应运动
        coord_type = self.mode_map.get(msg.mode, 0)
        self.get_logger().info(f'{self.robot_name} 收到{self.control_mode}指令: {target_pos[:6]}，执行中')
        self.execute_movement(target_pos, coord_type)
    
    def execute_movement(self, target_pos, coord_type):
        """执行运动指令"""
        if not self.connected:
            return
            
        try:
            print("start")
            # 确保机械臂处于就绪状态
            current_status = self.robot_arm.servo_status_inquire(1)
            while current_status != 3:
                self.robot_arm.operation_mode_set(0)
                self.robot_arm.fault_reset(1)
                self.robot_arm.servo_status_set(1, 1)
                self.robot_arm.deadman_status_set(0)
                self.robot_arm.deadman_status_set(1)
                self.robot_arm.servo_status_inquire(1)
                time.sleep(1)
                current_status = self.robot_arm.servo_status_inquire(1)

            # 发送运动指令（使用固定速度）
            coord_names = ['关节', '直角', '工具', '用户']
            self.get_logger().info(f'{self.robot_name} 发送{coord_names[coord_type]}运动指令，速度: {self.MOVEMENT_SPEED}%')
            self.robot_arm.movj(1, self.MOVEMENT_SPEED, coord_type, target_pos)
            self.get_logger().info(f'{self.robot_name} 运动指令已发送')
            
        except Exception as e:
            self.get_logger().error(f'{self.robot_name} 运动执行错误: {str(e)}')
    
    def update_robot_status(self):
        """更新机械臂状态并发布（包含故障检测）"""
        if not self.connected:
            self.status_pub.publish(String(data=f'{self.robot_name}: 未连接'))
            self.mode_pub.publish(String(data=f'{self.robot_name}: 控制模式: {self.control_mode}'))
            return
            
        try:
            # 获取并发布关节坐标（确保 joint_pos 不为 None）
            joint_pos = self.robot_arm.currentpos_inquiry(1, 0) or [0.0] * 7  # 默认值
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = [f'{self.robot_name}_joint_{i+1}' for i in range(7)]
            joint_state.position = joint_pos
            self.joint_state_pub.publish(joint_state)
            
            # 获取并发布直角坐标（确保 cartesian_pos 不为 None）
            cartesian_pos = self.robot_arm.currentpos_inquiry(1, 1) or [0.0] * 6  # 默认值
            cartesian_msg = Float64MultiArray()
            cartesian_msg.data = cartesian_pos
            self.cartesian_state_pub.publish(cartesian_msg)
            
            # 获取伺服状态（确保 servo_status 不为 None）
            servo_status = self.robot_arm.servo_status_inquire(1)
            status_map = {0: '停止', 1: '就绪', 2: '错误', 3: '运行'}
            servo_status_text = status_map.get(servo_status, "未知") if servo_status is not None else "未知"
            
            # 发布状态信息
            status_msg = (f'{self.robot_name}: 状态={servo_status_text}, '
                        f'控制模式={self.control_mode}, '
                        f'关节坐标: {[round(p, 2) for p in joint_pos]}, '
                        f'直角坐标: {[round(p, 2) for p in cartesian_pos[:6]]}')
            self.status_pub.publish(String(data=status_msg))
            self.mode_pub.publish(String(data=f'{self.robot_name}: 控制模式: {self.control_mode}'))
            
        except Exception as e:
            self.get_logger().error(f'{self.robot_name} 状态更新错误: {str(e)}')
            if not self.robot_arm.reconnect(retries=1):
                self.connected = False
                self.get_logger().error(f'{self.robot_name} 重新连接失败')
    
    def control_service_callback(self, request, response):
        """服务回调函数，处理字符串命令"""
        command = request.command  # 获取请求中的命令
        
        if not self.connected:
            response.is_command_success = 0
            response.message = f"{self.robot_name} 未连接，无法执行命令"
            self.get_logger().error(response.message)
            return response
            
        try:
            if command == "clear_warning":
                self.connect_to_robot()
                response.is_command_success = 1
                response.message = "success"
                self.get_logger().info(f"{self.robot_name} 警告已清除，系统已初始化")
                
            elif command == "read_motor_fault":
                # 执行读取电机故障操作
                fault_info = []
                for servo_num in range(1, 7):
                    error_code, _, _, _, _ = self.robot_arm.servo_inside_parm_inqure(1, servo_num)
                    
                    if error_code is None:
                        fault_info.append(f"电机{servo_num}状态未知（查询失败）")
                    elif error_code != 0:
                        fault_info.append(f"电机{servo_num}故障: 错误码={hex(error_code)}")
                
                if fault_info:
                    fault_msg = f"{self.robot_name} 故障: {'; '.join(fault_info)}"
                    self.fault_pub.publish(String(data=fault_msg))
                    response.is_command_success = 0
                    response.message = fault_msg
                    self.get_logger().warn(fault_msg)
                else:
                    msg = f"{self.robot_name} 无故障"
                    self.fault_pub.publish(String(data=msg))
                    response.is_command_success = 1
                    response.message = msg
                    self.get_logger().info(msg)
                
            elif command == "move_origin":
                # 执行回到原点操作
                self.get_logger().info(f"{self.robot_name} 执行回到原点命令")
                self.robot_arm.movJ(1, 50, 0, [280.0, 0.0, 400.0, 0.0, 0.0, 0.0, 0.0])
                response.is_command_success = 1
                response.message = "success"
                self.get_logger().info(f"{self.robot_name} 正在移动到原点位置")
                
            else:
                response.is_command_success = 0
                response.message = f"未知命令: {command}，支持的命令: clear_warning, read_motor_fault, move_origin"
                self.get_logger().warn(response.message)
                
        except Exception as e:
            response.is_command_success = 0
            response.message = f"执行命令 '{command}' 时出错: {str(e)}"
            self.get_logger().error(response.message)
            
        return response
    
    def destroy_node(self):
        """节点销毁时清理资源"""
        self.get_logger().info(f'{self.robot_name} 正在关闭连接...')
        if self.connected:
            # 停止运动
            self.robot_arm.servo_status_set(1, 0)
            time.sleep(0.5)
            # 下电
            self.robot_arm.deadman_status_set(0)
            time.sleep(0.5)
            # 断开连接
            self.robot_arm.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotArmController()
    
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        robot_controller.get_logger().info(f'{robot_controller.robot_name} 用户中断，关闭节点')
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()