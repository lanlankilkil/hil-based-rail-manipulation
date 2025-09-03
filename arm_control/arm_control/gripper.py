#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import Header

# 导入OmniState消息（请根据实际消息包路径修改）
from omni_msgs.msg import OmniState  # 需确保该消息已定义

from pymodbus.client import ModbusSerialClient
from math import ceil


class Communication:
    """串口通信类，负责与夹爪的底层通信"""
    def __init__(self):
        self.client = None
      
    def connect_to_device(self, device):
        """连接到设备，返回连接状态"""
        self.client = ModbusSerialClient(
            port=device,
            stopbits=1,
            bytesize=8,
            baudrate=115200,
            timeout=0.2
        )
        if not self.client.connect():
            print(f"无法连接到设备: {device}")
            return False
        return True

    def disconnect_from_device(self):
        """断开设备连接"""
        if self.client:
            self.client.close()

    def send_command(self, data):
        """发送控制命令到夹爪"""
        # 确保数据长度为偶数
        if len(data) % 2 == 1:
            data.append(0)

        message = []
        # 组合两个字节为一个寄存器值
        for i in range(0, int(len(data)/2)):
            message.append((data[2*i] << 8) + data[2*i+1])

        try:
            self.client.write_registers(0x03E8, message, slave=0x0009)
        except Exception as e:
            print(f"发送命令失败: {e}")

    def get_status(self, num_bytes):
        """读取夹爪状态"""
        num_regs = int(ceil(num_bytes / 2.0))
        try:
            response = self.client.read_holding_registers(
                address=0x07D0,
                count=num_regs,
                slave=0x09
            )
        except Exception as e:
            print(f"读取状态失败: {e}")
            return []

        output = []
        if hasattr(response, 'registers'):
            for reg in response.registers:
                output.append((reg & 0xFF00) >> 8)
                output.append(reg & 0x00FF)
        else:
            # 兼容老版本API
            for i in range(num_regs):
                reg = response.getRegister(i)
                output.append((reg & 0xFF00) >> 8)
                output.append(reg & 0x00FF)
        
        return output


class Robotiq2FGripper:
    """Robotiq 2F夹爪控制类"""
    def __init__(self, device, name):
        self.device = device
        self.name = name  # 夹爪名称（可通过参数设置）
        self.communication = Communication()
        self.command = {
            'rACT': 0,   # 激活状态
            'rGTO': 0,   # 运动使能
            'rATR': 0,   # 自动释放
            'rPR': 0,    # 目标位置
            'rSP': 255,  # 速度
            'rFR': 250   # 力度
        }

    def connect(self):
        """连接到夹爪"""
        return self.communication.connect_to_device(self.device)

    def disconnect(self):
        """断开连接"""
        self.communication.disconnect_from_device()

    def send_command(self):
        """发送命令帧"""
        data = [
            self.command['rACT'] + (self.command['rGTO'] << 3) + (self.command['rATR'] << 4),
            0,  # 保留位
            0,  # 保留位
            self.command['rPR'],
            self.command['rSP'],
            self.command['rFR']
        ]
        self.communication.send_command(data)

    def get_status(self):
        """获取夹爪状态"""
        status = self.communication.get_status(6)
        if len(status) < 6:
            return None
        return {
            'gACT': (status[0] >> 0) & 0x01,  # 激活状态
            'gGTO': (status[0] >> 3) & 0x01,  # 运动使能
            'gSTA': (status[0] >> 4) & 0x03,  # 工作状态
            'gOBJ': (status[0] >> 6) & 0x03,  # 物体检测
            'gFLT': status[2],                # 故障代码
            'gPR': status[3],                 # 目标位置
            'gPO': status[4],                 # 当前位置
            'gCU': status[5]                  # 当前电流
        }

    def print_status(self, prefix=""):
        """打印夹爪状态"""
        status = self.get_status()
        if not status:
            print(f"[{self.name}] {prefix}无法读取状态")
            return
        print(
            f"[{self.name}] {prefix}状态: "
            f"激活={status['gACT']}, 使能={status['gGTO']}, "
            f"状态={status['gSTA']}, 物体检测={status['gOBJ']}, "
            f"故障={status['gFLT']}, 目标位置={status['gPR']}, "
            f"当前位置={status['gPO']}"
        )

    def reset(self):
        """复位夹爪"""
        print(f"[{self.name}] 执行复位...")
        self.command = {
            'rACT': 0, 'rGTO': 0, 'rATR': 0,
            'rPR': 0, 'rSP': 255, 'rFR': 250
        }
        self.send_command()
        time.sleep(1)
        self.print_status("复位后")

    def activate(self):
        """激活夹爪"""
        print(f"[{self.name}] 执行激活...")
        self.command = {
            'rACT': 1, 'rGTO': 1, 'rATR': 0,
            'rPR': 0, 'rSP': 255, 'rFR': 250
        }
        self.send_command()
        time.sleep(1)
        self.print_status("激活后")

    def open(self):
        """打开夹爪"""
        print(f"[{self.name}] 打开夹爪...")
        self.command['rPR'] = 0  # 0对应完全打开
        self.send_command()
        time.sleep(1)
        self.print_status("打开后")

    def close(self):
        """关闭夹爪"""
        print(f"[{self.name}] 关闭夹爪...")
        self.command['rPR'] = 255  # 255对应完全关闭
        self.send_command()
        time.sleep(1)
        self.print_status("关闭后")


class RobotiqSingleGripperNode(Node):
    """ROS 2节点：单独控制一个夹爪，支持动态配置节点名和话题"""
    def __init__(self):
        # 节点名称通过外部参数传入，默认"robotiq_single_gripper"
        super().__init__('robotiq_single_gripper')
        
        # 声明可配置参数
        self.declare_parameter('gripper_name', 'gripper')  # 夹爪名称（用于区分）
        self.declare_parameter('device_path', '/dev/ttyUSB0')  # 串口设备路径
        self.declare_parameter('subscribe_topic', '/phantom/state')  # 串口设备路径

        # 获取参数值
        self.gripper_name = self.get_parameter('gripper_name').get_parameter_value().string_value
        self.device_path = self.get_parameter('device_path').get_parameter_value().string_value
        self.subscribe_topic = self.get_parameter('subscribe_topic').get_parameter_value().string_value

        # 创建夹爪实例
        self.gripper = Robotiq2FGripper(self.device_path, self.gripper_name)

        # 初始化夹爪
        self.initialize_gripper()

        # 订阅OmniState消息（话题名由参数指定）
        self.subscription = self.create_subscription(
            OmniState,
            self.subscribe_topic,
            self.omni_state_callback,
            10  # QoS设置
        )
        self.get_logger().info(
            f"节点初始化完成: 名称={self.get_name()}, "
            f"夹爪={self.gripper_name}, "
            f"串口={self.device_path}, "
            f"订阅话题={self.subscribe_topic}"
        )

    def initialize_gripper(self):
        """初始化夹爪（连接、复位、激活）"""
        if not self.gripper.connect():
            self.get_logger().error(f"{self.gripper_name}连接失败: {self.device_path}")
            return False
        self.get_logger().info(f"{self.gripper_name}已连接: {self.device_path}")
        self.gripper.reset()
        self.gripper.activate()
        return True

    def omni_state_callback(self, msg):
        """处理OmniState消息，控制当前夹爪"""
        if msg.close_gripper:
            self.gripper.close()
        else:
            self.gripper.open()

    def destroy_node(self):
        """节点销毁时清理资源"""
        self.gripper.disconnect()
        self.get_logger().info(f"{self.gripper_name}已断开连接")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    # 初始化节点（节点名称可通过外部参数覆盖）
    node = RobotiqSingleGripperNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，停止节点")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()