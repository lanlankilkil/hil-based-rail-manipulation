"""
Ironroad机械臂HTTP服务器
基于arm_control ROS2包装器，提供与franka_server.py相同的HTTP接口
"""
from flask import Flask, request, jsonify
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
import threading
from scipy.spatial.transform import Rotation as R
from absl import app, flags

from arm_control.msg import ControlCommand
from arm_control.srv import RobotArmCommand

FLAGS = flags.FLAGS
flags.DEFINE_string(
    "robot_name", "robot_left", "机械臂名称"
)
flags.DEFINE_string(
    "robot_ip", "192.168.1.13", "机械臂IP地址"
)
flags.DEFINE_integer(
    "robot_port", 6001, "机械臂端口"
)
flags.DEFINE_string(
    "flask_url", "127.0.0.1", "Flask服务器URL"
)
flags.DEFINE_integer(
    "flask_port", 5000, "Flask服务器端口"
)


class IronroadROS2Client(Node):
    """ROS2客户端，与arm_control节点通信"""
    
    def __init__(self, robot_name="robot_left"):
        super().__init__('ironroad_http_client')
        self.robot_name = robot_name
        
        # 发布器和服务客户端
        self.control_pub = self.create_publisher(
            ControlCommand,
            f'/{robot_name}/control_command',
            10
        )
        
        self.control_service_client = self.create_client(
            RobotArmCommand,
            f'/{robot_name}/control_service'
        )
        
        # 状态订阅
        self.joint_state_sub = self.create_subscription(
            JointState,
            f'/{robot_name}/joint_states',
            self._joint_state_callback,
            10
        )
        
        self.cartesian_state_sub = self.create_subscription(
            Float64MultiArray,
            f'/{robot_name}/cartesian_states', 
            self._cartesian_state_callback,
            10
        )
        
        # 状态存储
        self.current_joint_pos = [0.0] * 7
        self.current_cartesian_pos = [0.0] * 6
        self.last_cartesian_pos = [0.0] * 6
        self.last_update_time = time.time()
        
        # 夹爪状态（模拟）
        self.gripper_pos = 1.0
        
        self.get_logger().info(f'IronroadROS2Client 初始化完成，机械臂: {robot_name}')
    
    def _joint_state_callback(self, msg):
        """关节状态回调"""
        if len(msg.position) >= 7:
            self.current_joint_pos = list(msg.position[:7])
        
    def _cartesian_state_callback(self, msg):
        """直角坐标状态回调"""
        self.last_cartesian_pos = self.current_cartesian_pos.copy()
        self.current_cartesian_pos = list(msg.data[:6])
        self.last_update_time = time.time()
    
    def send_pose_command(self, pose):
        """发送位姿命令"""
        msg = ControlCommand()
        msg.mode = "cartesian"
        msg.position = pose
        self.control_pub.publish(msg)
        
    def send_joint_command(self, joints):
        """发送关节命令"""
        msg = ControlCommand()
        msg.mode = "joint" 
        msg.position = joints
        self.control_pub.publish(msg)
    
    def clear_errors(self):
        """清除错误"""
        if not self.control_service_client.wait_for_service(timeout_sec=1.0):
            return False
            
        request = RobotArmCommand.Request()
        request.command = "clear_warning"
        future = self.control_service_client.call_async(request)
        return True


class IronroadServer:
    """Ironroad机械臂服务器，提供HTTP接口"""
    
    def __init__(self, robot_name, robot_ip, robot_port):
        self.robot_name = robot_name
        self.robot_ip = robot_ip 
        self.robot_port = robot_port
        
        # 初始化ROS2
        rclpy.init()
        self.ros_client = IronroadROS2Client(robot_name)
        
        # 在单独线程中运行ROS2
        self.ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self.ros_thread.start()
        
        # 模拟Franka的状态格式
        self.pos = np.zeros(7)  # [x,y,z,qx,qy,qz,qw]
        self.vel = np.zeros(6) 
        self.force = np.zeros(3)  # arm_control不提供力传感器，设为0
        self.torque = np.zeros(3)  # arm_control不提供扭矩传感器，设为0
        self.q = np.zeros(7)
        self.dq = np.zeros(7)  # 通过数值微分估算
        self.jacobian = np.eye(6, 7)  # 简化的雅可比矩阵
        
        # 用于速度计算
        self.last_pos = np.zeros(7)
        self.last_time = time.time()
        
    def _spin_ros(self):
        """在单独线程中运行ROS2"""
        rclpy.spin(self.ros_client)
    
    def move(self, pose):
        """移动到指定位姿 [x,y,z,qx,qy,qz,qw]"""
        if len(pose) == 7:
            # 转换为arm_control格式 [x,y,z,a,b,c,0]
            xyz = pose[:3] * 1000  # 米转换为毫米
            quat = pose[3:]
            euler = R.from_quat(quat).as_euler('xyz')
            cartesian_pose = list(xyz) + list(np.degrees(euler)) + [0.0]
            self.ros_client.send_pose_command(cartesian_pose)
    
    def clear(self):
        """清除错误"""
        return self.ros_client.clear_errors()
    
    def update_state_from_ros(self):
        """从ROS2更新状态"""
        # 将arm_control的直角坐标转换为Franka格式
        if len(self.ros_client.current_cartesian_pos) >= 6:
            cart_pos = self.ros_client.current_cartesian_pos
            xyz = np.array(cart_pos[:3]) / 1000.0  # 毫米转米
            euler = np.radians(cart_pos[3:6])  # 度转弧度
            quat = R.from_euler('xyz', euler).as_quat()
            
            # 更新位置
            current_pos = np.concatenate([xyz, quat])
            
            # 计算速度（数值微分）
            current_time = time.time()
            dt = current_time - self.last_time
            if dt > 0:
                pos_diff = current_pos - self.last_pos
                self.vel[:3] = pos_diff[:3] / dt  # 线速度
                # 角速度计算（简化）
                if np.linalg.norm(pos_diff[3:]) > 0:
                    self.vel[3:] = pos_diff[3:] / dt
            
            self.last_pos = current_pos.copy()
            self.last_time = current_time
            self.pos = current_pos
        
        # 更新关节状态
        if len(self.ros_client.current_joint_pos) >= 7:
            new_q = np.array(self.ros_client.current_joint_pos)
            # 计算关节速度
            if hasattr(self, 'last_q_time'):
                dt = time.time() - self.last_q_time
                if dt > 0:
                    self.dq = (new_q - self.q) / dt
            self.q = new_q
            self.last_q_time = time.time()


def main(_):
    ROBOT_NAME = FLAGS.robot_name
    ROBOT_IP = FLAGS.robot_ip
    ROBOT_PORT = FLAGS.robot_port
    
    webapp = Flask(__name__)
    
    # 创建机械臂服务器
    robot_server = IronroadServer(ROBOT_NAME, ROBOT_IP, ROBOT_PORT)
    
    # 模拟夹爪服务器
    class MockGripperServer:
        def __init__(self):
            self.gripper_pos = 1.0
        
        def open(self):
            self.gripper_pos = 1.0
            print("Gripper opened")
            
        def close(self):
            self.gripper_pos = 0.0
            print("Gripper closed")
    
    gripper_server = MockGripperServer()
    
    # HTTP路由定义 - 兼容FrankaEnv的接口
    @webapp.route("/pose", methods=["POST"])
    def pose():
        pos = np.array(request.json["arr"])
        robot_server.move(pos)
        return "Moved"
    
    @webapp.route("/getstate", methods=["POST"])
    def get_state():
        robot_server.update_state_from_ros()
        return jsonify({
            "pose": robot_server.pos.tolist(),
            "vel": robot_server.vel.tolist(), 
            "force": robot_server.force.tolist(),
            "torque": robot_server.torque.tolist(),
            "q": robot_server.q.tolist(),
            "dq": robot_server.dq.tolist(),
            "jacobian": robot_server.jacobian.tolist(),
            "gripper_pos": gripper_server.gripper_pos,
        })
    
    @webapp.route("/clearerr", methods=["POST"])
    def clear():
        robot_server.clear()
        return "Clear"
        
    @webapp.route("/open_gripper", methods=["POST"])
    def open_gripper():
        gripper_server.open()
        return "Opened"
        
    @webapp.route("/close_gripper", methods=["POST"])
    def close_gripper():
        gripper_server.close()
        return "Closed"
    
    @webapp.route("/update_param", methods=["POST"])
    def update_param():
        # arm_control不需要动态参数更新，直接返回成功
        return "Updated parameters"
    
    print(f"Ironroad HTTP服务器启动: http://{FLAGS.flask_url}:{FLAGS.flask_port}")
    webapp.run(host=FLAGS.flask_url, port=FLAGS.flask_port)


if __name__ == "__main__":
    app.run(main)