#!/usr/bin/env python3
import sys
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray  # 修复未导入的消息类型
import time
import numpy as np

sys.path.insert(0, '/home/lz/DAMR/install/arm_control/lib/python3.10/site-packages')

# 导入自定义模块
from arm_control.robotarm_sdk import RobotArmSDK
from arm_control.msg import ControlCommand  # 自定义消息类型

class RobotArmController(Node):
    """机械臂控制节点，支持单一话题接收双模式控制指令"""
    
    def __init__(self):
        # super().__init__('robot_arm_controller')

        # 第一步：从命令行参数中提取节点名（不依赖 Node 类）
        # 解析启动时传递的 node_name 参数（默认值为 robot_arm_controller）
        import sys
        from rclpy.parameter import Parameter
        node_name = "robot_arm_controller"  # 默认节点名
        
        # 手动解析命令行参数中的 node_name（绕开未初始化的 Node 类）
        for arg in sys.argv:
            if arg.startswith('robot_name:='):
                node_name = arg.split(':=')[1]
                break

        # 第二步：必须先调用父类 Node 的构造函数（核心！）
        super().__init__(node_name)  # 这一行必须存在，且在所有参数声明前
        
        # 声明参数
        self.declare_parameter('robot_name', 'arm_left')
        self.declare_parameter('robot_ip', '192.168.1.13')
        self.declare_parameter('robot_port', 6001)
        self.declare_parameter('control_mode', 'joint')  # 默认关节控制模式
        
        # 获取参数值
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.robot_port = self.get_parameter('robot_port').get_parameter_value().integer_value
        self.control_mode = self.get_parameter('control_mode').get_parameter_value().string_value
        
        # 固定运动速度（取消参数化）
        self.MOVEMENT_SPEED = 50  # 固定速度50%
        
        # 初始化机械臂SDK
        self.robot_arm = RobotArmSDK(self.robot_ip, self.robot_port)
        self.connected = False
        
        # 控制模式映射
        self.mode_map = {
            'joint': 0,    # 关节坐标
            'cartesian': 1 # 直角坐标
        }
        
        # 构建话题名称
        self.command_topic = f'/{self.robot_name}/control_command'  # 合并后的命令话题
        self.joint_state_topic = f'/{self.robot_name}/joint_states'
        self.cartesian_state_topic = f'/{self.robot_name}/cartesian_states'
        self.status_topic = f'/{self.robot_name}/robot_status'
        self.mode_topic = f'/{self.robot_name}/control_mode'
        self.fault_topic = f'/{self.robot_name}/fault_status'
        
        # 连接机械臂
        self.connect_to_robot()
        
        # ROS 订阅者和发布者
        self.command_sub = self.create_subscription(
            ControlCommand,  # 使用自定义消息类型
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
            self.fault_topic,  # 故障状态话题
            10)
        
        # 参数动态更新回调
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # 状态更新定时器
        self.status_timer = self.create_timer(0.1, self.update_robot_status)
        
        self.get_logger().info(f'{self.robot_name} 控制节点已启动')
        self.get_logger().info(f'当前控制模式: {self.control_mode}，固定速度: {self.MOVEMENT_SPEED}%')
    
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
            
            if self.robot_arm.servo_status_inquire(1) == 0:
                # 设置伺服状态为就绪
                self.robot_arm.servo_status_set(1, 1)
                time.sleep(0.5)

            # 初始化机械臂状态
            time.sleep(1)
            self.robot_arm.fault_reset(1)
            time.sleep(0.5)

            if self.robot_arm.deadman_status_inquire() != 1:
                # 设置上电状态
                self.robot_arm.deadman_status_set(1)
                time.sleep(0.5)
            
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
    
    def command_callback(self, msg):
        """处理合并后的控制指令"""

        if not self.connected:
            self.get_logger().warning(f'{self.robot_name} 未连接，无法执行指令')
            return
            
        # 检查指令模式是否与当前控制模式匹配
        if msg.mode != self.control_mode:
            self.get_logger().info(f'{self.robot_name} 收到{msg.mode}指令，但当前模式为 {self.control_mode}，忽略')
            return
            
        # 验证位置数据长度
        if len(msg.position) != 7:
            self.get_logger().error(f'{self.robot_name} 无效指令长度: {len(msg.position)}, 期望7个值')
            return
            
        target_pos = list(msg.position)
        
        # 根据控制模式执行相应运动
        coord_type = self.mode_map.get(self.control_mode, 0)
        self.get_logger().info(f'{self.robot_name} 收到{self.control_mode}指令: {target_pos[:6]}，执行中')
        self.execute_movement(target_pos, coord_type)
    
    def execute_movement(self, target_pos, coord_type):
        """执行运动指令"""
        if not self.connected:
            return
            
        try:
            # 确保机械臂处于就绪状态
            current_status = self.robot_arm.servo_status_inquire(1)
            if current_status != 1:
                self.robot_arm.servo_status_set(1, 1)
                time.sleep(0.5)
            
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
            
            # 查询6个电机的故障状态（1-6号电机）
            fault_info = []
            for servo_num in range(1, 7):
                # 调用伺服参数查询接口
                error_code, _, _, _, _ = self.robot_arm.servo_inside_parm_inqure(1, servo_num)
                
                # 处理 None 类型和非零错误码
                if error_code is None:
                    fault_info.append(f"电机{servo_num}状态未知（查询失败）")
                elif error_code != 0:
                    fault_info.append(f"电机{servo_num}故障: 错误码={hex(error_code)}")
            
            # 发布故障信息
            if fault_info:
                fault_msg = String(data=f"{self.robot_name} 故障: {'; '.join(fault_info)}")
                self.fault_pub.publish(fault_msg)
                self.get_logger().warn(fault_msg.data)
            else:
                self.fault_pub.publish(String(data=f"{self.robot_name} 无故障"))
            
            # 获取伺服状态（确保 servo_status 不为 None）
            servo_status = self.robot_arm.servo_status_inquire(1)
            status_map = {0: '停止', 1: '就绪', 2: '错误', 3: '运行'}
            servo_status_text = status_map.get(servo_status, "未知") if servo_status is not None else "未知"
            
            # 发布状态信息（所有数值都使用默认值确保不为 None）
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
