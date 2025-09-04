# Pick Ironroad 部署指南

## 概述
pick_ironroad是基于USB Pick Up and Insertion改造的新机械臂部署方案，使用arm_control（ROS2）替代原有的Franka（ROS1）控制系统。**专注于已训练模型的部署**，采用直接ROS2通信方式，集成真实夹爪控制。

## 快速部署（推荐）

### 1. 一键部署已训练模型
```bash
# 切换到项目根目录
cd /home/wang/Project/hil-serl

# 部署指定检查点
bash examples1/experiments/pick_ironroad/run_actor.sh --eval_checkpoint_step=10000 --eval_n_trajs=10

# 或使用独立部署脚本
python examples1/experiments/pick_ironroad/deploy.py --checkpoint_path=./checkpoints/your_model
```

### 2. 测试系统连接（无需模型）
```bash
python examples1/experiments/pick_ironroad/deploy.py --test_mode
```

## 部署架构

```
训练好的RL模型 (.pkl/.jax检查点)
    ↓ (加载参数)
deploy.py (独立部署脚本)
    ↓ (创建环境)
IronroadEnv (ROS2直通)
    ↓ (ROS2话题通信)
arm_control节点 + gripper节点
    ↓ (硬件控制)
机械臂硬件 + Robotiq夹爪
```

## 部署方式对比

| 方式 | 适用场景 | 优势 | 命令 |
|------|----------|------|------|
| **run_actor.sh --eval** | 正式部署评估 | 完整HIL-SERL框架 | `bash run_actor.sh --eval_checkpoint_step=N` |
| **deploy.py** | 独立部署测试 | 简洁直接，易调试 | `python deploy.py --checkpoint_path=path` |
| **deploy.py --test_mode** | 硬件连接测试 | 无需训练模型 | `python deploy.py --test_mode` |

## 硬件连接检查

### 启动arm_control系统
```bash
# 手动启动（用于调试）
ros2 launch arm_control left_arm_control.launch.py

# 检查节点状态
ros2 node list | grep -E "robot_left|gripper_left"
```

### 验证通信
```bash
# 检查机械臂话题
ros2 topic echo /robot_left/cartesian_states

# 检查夹爪话题
ros2 topic echo /phantom1/state
```

## 训练vs部署

| 阶段 | 目的 | 脚本 | 硬件要求 |
|------|------|------|----------|
| **训练** | 收集数据、学习策略 | `run_actor.sh` + `run_learner.sh` | 完整硬件 + 人工干预 |
| **部署** | 执行训练好的策略 | `deploy.py` 或 `run_actor.sh --eval` | 完整硬件（无需人工） |

## 部署前置条件

### 必需组件
1. **ROS2环境** - source你的ROS2安装
2. **arm_control包** - 确保已编译安装  
3. **训练检查点** - .pkl或.jax格式的模型文件
4. **硬件连接** - 机械臂网络连接 + 夹爪串口 + 相机USB

### 检查清单
- [ ] ROS2环境已source
- [ ] arm_control节点能正常启动
- [ ] 机械臂网络连接正常（192.168.1.13:6001）
- [ ] 夹爪串口设备存在（/dev/gripper_left）
- [ ] 相机USB连接并可识别（rs-enumerate-devices）
- [ ] 训练检查点文件路径正确

## 故障排除

### 部署失败常见问题
1. **模型加载失败** - 检查检查点文件路径和格式
2. **ROS2节点连接失败** - 确认arm_control系统正常运行
3. **机械臂无响应** - 检查网络连接和arm_control配置
4. **夹爪控制失败** - 验证串口设备和权限
5. **相机图像获取失败** - 检查USB连接和RealSense驱动

### 调试命令
```bash
# 检查ROS2节点
ros2 node list

# 检查话题通信  
ros2 topic list | grep -E "robot_left|gripper|phantom1"

# 测试相机
rs-enumerate-devices

# 查看日志
ros2 launch arm_control left_arm_control.launch.py --ros-args --log-level debug
```

## 配置说明

### 文件结构
```
examples1/experiments/pick_ironroad/
├── config.py              # 环境和训练配置
├── wrapper.py              # IronroadEnv环境类 + ROS2节点
├── deploy.py               # 模型部署脚本
├── run_actor.sh           # Actor脚本（训练交互/评估部署）
├── run_learner.sh         # Learner脚本（强化学习训练）
└── README.md              # 本文档
```

### 关键配置项（config.py）
```python
# 机械臂和夹爪节点名称
ROBOT_NAME = "robot_left"
GRIPPER_NAME = "gripper_left" 

# 任务目标位姿（毫米 + 度）
TARGET_POSE = np.array([400, 100, 200, 0, 180, 0])  # 抓取目标

# 重置位姿（毫米 + 度）  
RESET_POSE = np.array([300, 0, 300, 0, 180, 0])    # 初始位置

# 动作缩放
ACTION_SCALE = np.array([0.01, 0.1, 1])  # [位置, 姿态, 夹爪]

# 安全边界（毫米 + 度）
ABS_POSE_LIMIT_LOW = np.array([200, -200, 100, -45, 135, -45])
ABS_POSE_LIMIT_HIGH = np.array([600, 200, 400, 45, 225, 45])

# 相机配置（复用franka_env）
REALSENSE_CAMERAS = {
    "wrist_cam": {"serial_number": "138422073424", "dim": (640, 480)}
}
```

### 坐标系转换
```python
# 训练时格式（IronroadEnv内部使用）
currpos = [x, y, z, qx, qy, qz, qw]  # 米 + 四元数

# arm_control格式（发送给硬件）
cartesian_pose = [x, y, z, a, b, c, 0]  # 毫米 + 欧拉角(度)

# 自动转换代码（wrapper.py:386-390）
xyz = pos[:3] * 1000  # 米转毫米
euler = Rotation.from_quat(pos[3:]).as_euler('xyz')  # 四元数转欧拉角
cartesian_pose = list(xyz) + list(np.degrees(euler)) + [0.0]  # 弧度转度
```

## 核心代码架构

### IronroadROS2Node类（wrapper.py）
```python
class IronroadROS2Node(Node):
    def __init__(self, robot_name="robot_left"):
        super().__init__('ironroad_env_client')
        
        # 机械臂控制
        self.control_pub = self.create_publisher(
            ControlCommand, f'/{robot_name}/control_command', 10)
        
        # 夹爪控制（通过OmniState消息）
        self.gripper_pub = self.create_publisher(
            OmniState, '/phantom1/state', 10)
        
        # 状态订阅
        self.joint_state_sub = self.create_subscription(
            JointState, f'/{robot_name}/joint_states', 
            self._joint_state_callback, 10)
    
    def send_pose_command(self, pose):
        """发送位姿命令到arm_control"""
        msg = ControlCommand()
        msg.mode = "cartesian"
        msg.position = pose  # [x,y,z,a,b,c,0]
        self.control_pub.publish(msg)
    
    def send_gripper_command(self, open_gripper: bool):
        """控制夹爪开合"""
        msg = OmniState()
        msg.close_gripper = not open_gripper
        self.gripper_pub.publish(msg)
```

### IronroadEnv类（wrapper.py）
```python
class IronroadEnv(gym.Env):
    def step(self, action: np.ndarray):
        # 位置增量控制
        xyz_delta = action[:3]
        nextpos = self.currpos.copy()
        nextpos[:3] += xyz_delta * self.action_scale[0]
        
        # 姿态增量控制  
        nextpos[3:] = (Rotation.from_euler("xyz", action[3:6] * self.action_scale[1])
                      * Rotation.from_quat(self.currpos[3:])).as_quat()
        
        # 夹爪控制
        gripper_action = action[6] * self.action_scale[2]
        self._send_gripper_command(gripper_action)
        
        # 发送位姿命令（自动转换坐标系）
        self._send_pos_command(self.clip_safety_box(nextpos))
        
        # 获取观测和奖励
        ob = self._get_obs()
        reward = self.compute_reward(ob)
        done = self.curr_path_length >= self.max_episode_length or reward
        
        return ob, int(reward), done, False, {"succeed": reward}
```

### 部署脚本（deploy.py）
```python
def deploy_model(agent, checkpoint_path=None):
    # 初始化ROS2和环境
    if not rclpy.ok():
        rclpy.init()
    
    config = TrainConfig()
    env = config.get_environment(fake_env=False)  # 创建真实环境
    
    while True:
        obs, info = env.reset()  # 重置到初始位置
        done = False
        
        while not done:
            if agent:
                action = agent.predict(obs, deterministic=True)  # 模型预测
            else:
                action = env.action_space.sample() * 0.1  # 测试模式
            
            obs, reward, done, truncated, info = env.step(action)
            if reward:
                print("任务完成!")
                break
```

## 与原版差异

| 项目 | 原版Franka | 新版Ironroad |
|------|------------|--------------|
| **部署方式** | HTTP服务器 | 直接ROS2 |
| **延迟** | 高（HTTP转换） | 低（直接通信） |
| **硬件** | Franka专用 | arm_control通用 |
| **训练接口** | 相同gym接口 | **完全兼容** |

## 优势
- **即插即用**: 训练好的模型可直接部署到新硬件
- **低延迟**: 去除HTTP中间层，实现实时控制  
- **通用性**: 支持所有arm_control兼容的机械臂
- **稳定性**: 基于ROS2的可靠通信机制

## 硬件要求

### 机械臂：
- 支持arm_control SDK的机械臂
- 网络连接 (如192.168.1.13:6001)

### 夹爪：
- Robotiq 2F夹爪
- USB转串口适配器 (如/dev/gripper_left)

### 相机：
- Intel RealSense相机 (D415/D435等)
- USB 3.0连接
- 已知相机序列号

## 故障排除

### ROS2相关：
1. **节点无法连接机械臂**: 检查`robot_ip`和`robot_port`配置
2. **状态订阅失败**: 确认arm_control节点正常运行
3. **模型加载失败**: 检查检查点文件路径和格式

### 夹爪相关：
4. **夹爪无响应**: 检查`/dev/gripper_left`设备路径
5. **Modbus通信失败**: 确认串口设备权限和连接
6. **夹爪初始化失败**: 检查gripper节点启动状态

### 相机相关：
7. **相机无法识别**: 检查RealSense驱动安装
8. **序列号错误**: 使用`rs-enumerate-devices`查看可用相机
9. **USB带宽不足**: 降低分辨率或帧率配置

## 优势
- **更低延迟**: 直接ROS2通信，无HTTP转换
- **更高实时性**: 状态自动推送，非查询式
- **真实硬件**: 集成真实Robotiq夹爪控制
- **相机复用**: USB直连RealSense，无需重新实现
- **架构简化**: 减少中间层，提高稳定性