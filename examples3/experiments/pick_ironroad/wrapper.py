from typing import OrderedDict, Dict
import numpy as np
import copy
import gymnasium as gym
import time
import cv2
import queue
import threading
from datetime import datetime
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from arm_control.msg import ControlCommand
from arm_control.srv import RobotArmCommand
from omni_msgs.msg import OmniState


class IronroadROS2Node(Node):
    """ROS2节点，直接与arm_control通信"""
    
    def __init__(self, robot_name="robot_left"):
        super().__init__('ironroad_env_client')
        self.robot_name = robot_name
        
        # 发布器
        self.control_pub = self.create_publisher(
            ControlCommand,
            f'/{robot_name}/control_command',
            10
        )
        
        # 服务客户端
        self.control_service_client = self.create_client(
            RobotArmCommand,
            f'/{robot_name}/control_service'
        )
        
        # 夹爪控制发布器（通过OmniState消息）
        gripper_name = robot_name.replace("robot_", "gripper_")
        self.gripper_topic = f'/phantom1/state'  # 夹爪监听的话题
        self.gripper_pub = self.create_publisher(
            OmniState,
            self.gripper_topic,
            10
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
        self.last_joint_pos = [0.0] * 7
        self.last_update_time = time.time()
        self.gripper_pos = 1.0
        
    def _joint_state_callback(self, msg):
        """关节状态回调"""
        if len(msg.position) >= 7:
            self.last_joint_pos = self.current_joint_pos.copy()
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
    
    def clear_errors(self):
        """清除错误"""
        if not self.control_service_client.wait_for_service(timeout_sec=1.0):
            return False
            
        request = RobotArmCommand.Request()
        request.command = "clear_warning"
        future = self.control_service_client.call_async(request)
        return True
    
    def send_gripper_command(self, open_gripper: bool):
        """通过OmniState消息控制夹爪"""
        msg = OmniState()
        msg.close_gripper = not open_gripper  # close_gripper=True表示关闭
        self.gripper_pub.publish(msg)
        
        # 更新夹爪状态
        self.gripper_pos = 1.0 if open_gripper else 0.0
        return True


class IronroadEnv(gym.Env):
    """基于arm_control的机械臂环境，直接使用ROS2通信"""
    
    def __init__(self, hz=10, fake_env=False, save_video=False, config=None):
        self.action_scale = config.ACTION_SCALE
        self._TARGET_POSE = config.TARGET_POSE
        self._RESET_POSE = config.RESET_POSE
        self.config = config
        self.max_episode_length = config.MAX_EPISODE_LENGTH
        self.display_image = config.DISPLAY_IMAGE
        
        # 初始化ROS2
        if not fake_env:
            if not rclpy.ok():
                rclpy.init()
            self.ros_node = IronroadROS2Node(getattr(config, 'ROBOT_NAME', 'robot_left'))
            self._ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
            self._ros_thread.start()
        else:
            self.ros_node = None
        
        # 转换位姿格式（arm_control使用毫米和度）
        self.target_pose_mm = np.concatenate([
            config.TARGET_POSE[:3], 
            config.TARGET_POSE[3:],
            [0.0]
        ])
        self.reset_pose_mm = np.concatenate([
            config.RESET_POSE[:3],
            config.RESET_POSE[3:], 
            [0.0]
        ])
        
        # 当前状态
        # 初始化为有效的位姿：[x,y,z,qx,qy,qz,qw]，四元数为单位四元数[0,0,0,1]
        self.currpos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])  # [x,y,z,qx,qy,qz,qw]
        self.currvel = np.zeros(6)
        self.currforce = np.zeros(3)
        self.currtorque = np.zeros(3)
        self.q = np.zeros(7)
        self.dq = np.zeros(7)
        self.currjacobian = np.eye(6, 7)
        self.curr_gripper_pos = np.array([1.0])
        
        self.randomreset = config.RANDOM_RESET
        self.random_xy_range = config.RANDOM_XY_RANGE / 1000.0  # 毫米转米
        self.random_rz_range = config.RANDOM_RZ_RANGE
        self.hz = hz
        
        # 动作和观测空间（保持与FrankaEnv相同）
        self.action_space = gym.spaces.Box(
            np.ones((7,), dtype=np.float32) * -1,
            np.ones((7,), dtype=np.float32),
        )
        
        # 安全边界（转换为米）
        self.xyz_bounding_box = gym.spaces.Box(
            config.ABS_POSE_LIMIT_LOW[:3] / 1000.0,
            config.ABS_POSE_LIMIT_HIGH[:3] / 1000.0,
            dtype=np.float64,
        )
        self.rpy_bounding_box = gym.spaces.Box(
            config.ABS_POSE_LIMIT_LOW[3:],
            config.ABS_POSE_LIMIT_HIGH[3:],
            dtype=np.float64,
        )
        
        self.observation_space = gym.spaces.Dict({
            "state": gym.spaces.Dict({
                "tcp_pose": gym.spaces.Box(-np.inf, np.inf, shape=(7,)),
                "tcp_vel": gym.spaces.Box(-np.inf, np.inf, shape=(6,)),
                "gripper_pose": gym.spaces.Box(-1, 1, shape=(1,)),
                "tcp_force": gym.spaces.Box(-np.inf, np.inf, shape=(3,)),
                "tcp_torque": gym.spaces.Box(-np.inf, np.inf, shape=(3,)),
            }),
            "images": gym.spaces.Dict({
                key: gym.spaces.Box(0, 255, shape=(128, 128, 3), dtype=np.uint8) 
                for key in config.REALSENSE_CAMERAS
            }) if hasattr(config, 'REALSENSE_CAMERAS') else gym.spaces.Dict()
        })
        
        self.curr_path_length = 0
        self.terminate = False
        
        # 初始化相机变量
        self.cap = None
        
        if fake_env:
            return
            
        # 初始化相机（如果配置了的话）
        if hasattr(config, 'REALSENSE_CAMERAS'):
            self.init_cameras(config.REALSENSE_CAMERAS)
            
        self.save_video = save_video
        if self.save_video:
            self.recording_frames = []
            
        # 等待ROS2节点就绪
        time.sleep(1.0)
        print("Initialized Ironroad Arm with ROS2")
    
    def _spin_ros(self):
        """在单独线程中运行ROS2"""
        rclpy.spin(self.ros_node)

    def init_cameras(self, name_serial_dict):
        """初始化相机"""
        if self.cap is not None:
            self.close_cameras()
            
        self.cap = OrderedDict()
        for cam_name, kwargs in name_serial_dict.items():
            try:
                from franka_env.camera.video_capture import VideoCapture
                from franka_env.camera.rs_capture import RSCapture
                cap = VideoCapture(RSCapture(name=cam_name, **kwargs))
                self.cap[cam_name] = cap
            except ImportError:
                print(f"警告：无法导入相机模块，跳过相机 {cam_name}")

    def close_cameras(self):
        """关闭相机"""
        if self.cap:
            for cap in self.cap.values():
                try:
                    cap.close()
                except Exception as e:
                    print(f"关闭相机失败: {e}")

    def clip_safety_box(self, pose: np.ndarray) -> np.ndarray:
        """安全边界限制"""
        pose_clipped = pose.copy()
        pose_clipped[:3] = np.clip(
            pose_clipped[:3], self.xyz_bounding_box.low, self.xyz_bounding_box.high
        )
        
        # 姿态限制
        euler = Rotation.from_quat(pose_clipped[3:]).as_euler("xyz")
        euler = np.clip(euler, self.rpy_bounding_box.low, self.rpy_bounding_box.high)
        pose_clipped[3:] = Rotation.from_euler("xyz", euler).as_quat()
        
        return pose_clipped

    def step(self, action: np.ndarray) -> tuple:
        """执行动作"""
        start_time = time.time()
        action = np.clip(action, self.action_space.low, self.action_space.high)
        
        # 位置增量
        xyz_delta = action[:3]
        
        self._update_currpos()
        nextpos = self.currpos.copy()
        nextpos[:3] = nextpos[:3] + xyz_delta * self.action_scale[0]
        
        # 姿态增量
        nextpos[3:] = (
            Rotation.from_euler("xyz", action[3:6] * self.action_scale[1])
            * Rotation.from_quat(self.currpos[3:])
        ).as_quat()
        
        # 夹爪控制
        gripper_action = action[6] * self.action_scale[2]
        self._send_gripper_command(gripper_action)
        
        # 发送位姿命令
        self._send_pos_command(self.clip_safety_box(nextpos))
        
        self.curr_path_length += 1
        
        # 控制频率
        dt = time.time() - start_time
        time.sleep(max(0, (1.0 / self.hz) - dt))
        
        # 获取观测
        self._update_currpos()
        ob = self._get_obs()
        reward = self.compute_reward(ob)
        done = self.curr_path_length >= self.max_episode_length or reward or self.terminate
        
        return ob, int(reward), done, False, {"succeed": reward}

    def compute_reward(self, obs) -> bool:
        """计算奖励（到达目标判断）"""
        current_pose = obs["state"]["tcp_pose"]
        target_pose_m = self._TARGET_POSE.copy()
        target_pose_m[:3] = target_pose_m[:3] / 1000.0  # 转换为米
        
        # 位置差异
        pos_diff = np.abs(current_pose[:3] - target_pose_m[:3])
        
        # 姿态差异
        current_rot = Rotation.from_quat(current_pose[3:]).as_matrix()
        target_rot = Rotation.from_euler("xyz", target_pose_m[3:]).as_matrix()
        diff_rot = current_rot.T @ target_rot
        diff_euler = Rotation.from_matrix(diff_rot).as_euler("xyz")
        rot_diff = np.abs(diff_euler)
        
        # 阈值检查（需要根据任务调整）
        pos_threshold = 0.005  # 5mm
        rot_threshold = 0.05   # ~3度
        
        if np.all(pos_diff < pos_threshold) and np.all(rot_diff < rot_threshold):
            return True
        return False

    def reset(self, **kwargs):
        """重置环境"""
        self._recover()
        self.curr_path_length = 0
        self.terminate = False
        
        # 移动到重置位置
        if self.randomreset:
            reset_pose = self._RESET_POSE.copy()
            reset_pose[:2] += np.random.uniform(-self.random_xy_range, self.random_xy_range, (2,)) * 1000
            reset_pose[5] += np.random.uniform(-self.random_rz_range, self.random_rz_range)
        else:
            reset_pose = self._RESET_POSE.copy()
            
        self.interpolate_move(reset_pose, timeout=1.0)
        
        # 开启夹爪
        self._send_gripper_command(1.0)
        time.sleep(0.5)
        
        self._update_currpos()
        obs = self._get_obs()
        return obs, {"succeed": False}

    def interpolate_move(self, goal_mm: np.ndarray, timeout: float):
        """插值移动（输入为毫米和度）"""
        if not self.ros_node:
            return
            
        # 直接使用arm_control格式 [x,y,z,a,b,c,0]
        goal_arm_control = np.concatenate([goal_mm, [0.0]])
        self.ros_node.send_pose_command(goal_arm_control)
        time.sleep(timeout)
        self._update_currpos()

    def get_im(self) -> Dict[str, np.ndarray]:
        """获取图像"""
        images = {}
        if not self.cap:
            # 如果没有相机，返回虚拟图像
            for key in getattr(self.config, 'REALSENSE_CAMERAS', {}):
                images[key] = np.zeros((128, 128, 3), dtype=np.uint8)
            return images
            
        for key, cap in self.cap.items():
            try:
                rgb = cap.read()
                if key in self.config.IMAGE_CROP:
                    cropped_rgb = self.config.IMAGE_CROP[key](rgb)
                else:
                    cropped_rgb = rgb
                resized = cv2.resize(cropped_rgb, (128, 128))
                images[key] = resized[..., ::-1]  # BGR to RGB
            except:
                images[key] = np.zeros((128, 128, 3), dtype=np.uint8)
        return images

    def _recover(self):
        """恢复机械臂状态"""
        if self.ros_node:
            self.ros_node.clear_errors()

    def _send_pos_command(self, pos: np.ndarray):
        """发送位姿命令"""
        if not self.ros_node:
            return
            
        self._recover()
        # 转换为arm_control格式 [x,y,z,a,b,c,0]
        xyz = pos[:3] * 1000  # 米转毫米
        quat = pos[3:]
        euler = Rotation.from_quat(quat).as_euler('xyz')
        cartesian_pose = list(xyz) + list(np.degrees(euler)) + [0.0]
        
        self.ros_node.send_pose_command(cartesian_pose)

    def _send_gripper_command(self, pos: float):
        """发送夹爪命令"""
        if not self.ros_node:
            return
            
        if pos >= 0.5:
            # 打开夹爪
            success = self.ros_node.send_gripper_command(True)
            if success:
                self.curr_gripper_pos = np.array([1.0])
                print("Gripper opened")
        elif pos <= -0.5:
            # 关闭夹爪
            success = self.ros_node.send_gripper_command(False)
            if success:
                self.curr_gripper_pos = np.array([0.0])
                print("Gripper closed")

    def _update_currpos(self):
        """从ROS2更新当前位姿"""
        if not self.ros_node:
            return
            
        try:
            # 从arm_control获取直角坐标
            if len(self.ros_node.current_cartesian_pos) >= 6:
                cart_pos = self.ros_node.current_cartesian_pos
                xyz = np.array(cart_pos[:3]) / 1000.0  # 毫米转米
                euler = np.radians(cart_pos[3:6])  # 度转弧度
                quat = Rotation.from_euler('xyz', euler).as_quat()
                self.currpos = np.concatenate([xyz, quat])
            
            # 计算速度（数值微分）
            current_time = time.time()
            dt = current_time - getattr(self, 'last_pos_time', current_time)
            if dt > 0 and hasattr(self, 'last_pos'):
                pos_diff = self.currpos - self.last_pos
                self.currvel[:3] = pos_diff[:3] / dt  # 线速度
                if np.linalg.norm(pos_diff[3:]) > 0:
                    self.currvel[3:] = pos_diff[3:] / dt  # 角速度（简化）
            
            self.last_pos = self.currpos.copy()
            self.last_pos_time = current_time
            
            # 更新关节状态
            if len(self.ros_node.current_joint_pos) >= 7:
                new_q = np.array(self.ros_node.current_joint_pos)
                if hasattr(self, 'last_q_time'):
                    dt_q = current_time - self.last_q_time
                    if dt_q > 0:
                        self.dq = (new_q - self.q) / dt_q
                self.q = new_q
                self.last_q_time = current_time
            
            # arm_control不提供力/扭矩传感器，设为0
            self.currforce = np.zeros(3)
            self.currtorque = np.zeros(3)
            self.currjacobian = np.eye(6, 7)  # 简化的雅可比
            self.curr_gripper_pos = np.array([self.ros_node.gripper_pos])
            
        except Exception as e:
            print(f"ROS2状态更新失败: {e}")

    def _get_obs(self) -> dict:
        """获取观测"""
        images = self.get_im()
        state_observation = {
            "tcp_pose": self.currpos,
            "tcp_vel": self.currvel,
            "gripper_pose": self.curr_gripper_pos,
            "tcp_force": self.currforce,
            "tcp_torque": self.currtorque,
        }
        return copy.deepcopy(dict(images=images, state=state_observation))

    def close(self):
        """关闭环境"""
        if hasattr(self, 'cap') and self.cap:
            self.close_cameras()
        
        # 关闭ROS2节点
        if hasattr(self, 'ros_node') and self.ros_node:
            self.ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class GripperPenaltyWrapper(gym.Wrapper):
    """夹爪惩罚包装器"""
    def __init__(self, env, penalty=-0.05):
        super().__init__(env)
        assert env.action_space.shape == (7,)
        self.penalty = penalty
        self.last_gripper_pos = None

    @property 
    def max_episode_length(self):
        """代理到基础环境的max_episode_length属性"""
        return getattr(self.env, 'max_episode_length', 120)

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        # 由于SERLObsWrapper的处理，state现在是numpy数组
        # 需要从原始环境获取gripper位置信息
        if hasattr(self.env, 'unwrapped'):
            # 获取最底层的环境
            base_env = self.env.unwrapped
            if hasattr(base_env, 'curr_gripper_pos'):
                gripper_pos = base_env.curr_gripper_pos
                self.last_gripper_pos = gripper_pos[0] if hasattr(gripper_pos, '__getitem__') else float(gripper_pos)
            else:
                self.last_gripper_pos = 1.0
        else:
            self.last_gripper_pos = 1.0
        return obs, info

    def step(self, action):
        observation, reward, terminated, truncated, info = self.env.step(action)
        
        if "intervene_action" in info:
            action = info["intervene_action"]

        # 夹爪惩罚逻辑
        if (action[-1] < -0.5 and self.last_gripper_pos > 0.9) or (
            action[-1] > 0.5 and self.last_gripper_pos < 0.9
        ):
            info["grasp_penalty"] = self.penalty
        else:
            info["grasp_penalty"] = 0.0

        # 安全地更新夹爪位置
        if hasattr(self.env, 'unwrapped'):
            base_env = self.env.unwrapped
            if hasattr(base_env, 'curr_gripper_pos'):
                gripper_pos = base_env.curr_gripper_pos
                self.last_gripper_pos = gripper_pos[0] if hasattr(gripper_pos, '__getitem__') else float(gripper_pos)
            else:
                self.last_gripper_pos = 1.0
        else:
            self.last_gripper_pos = 1.0
        return observation, reward, terminated, truncated, info
