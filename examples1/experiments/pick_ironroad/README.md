# Pick Ironroad 部署指南

## 概述
pick_ironroad是基于USB Pick Up and Insertion改造的新机械臂部署方案，使用arm_control（ROS2）替代原有的Franka（ROS1）控制系统。采用直接ROS2通信方式，集成真实夹爪控制。

## 系统架构

```
已训练的强化学习模型 
    ↓ (预测动作)
IronroadEnv (gym接口)
    ↓ (ROS2消息/话题)
IronroadROS2Node 
    ├─→ arm_control节点 (机械臂位姿控制)
    └─→ gripper节点 (夹爪控制)
        ↓ (Modbus串口)
    Robotiq 2F夹爪硬件
```

## 硬件连接

| 设备 | 连接方式 | 通信协议 |
|------|----------|----------|
| **机械臂** | 网络 | TCP/IP + arm_control SDK |
| **夹爪** | USB转串口 | Modbus RTU |
| **相机** | USB直连 | Intel RealSense API |

## 文件结构

```
examples/experiments/pick_ironroad/
├── config.py              # 环境和训练配置
├── wrapper.py              # IronroadEnv环境类 + ROS2节点
├── deploy.py               # 模型部署脚本
├── start_deployment.sh     # 启动脚本
└── README.md              # 本文档
```

## 部署步骤

### 1. 启动完整的arm_control系统
```bash
# 启动机械臂 + 夹爪节点
ros2 launch arm_control left_arm_control.launch.py
```

这会启动：
- `robot_left_arm` - 机械臂控制节点
- `gripper_left` - 夹爪控制节点

### 2. 直接部署推理模型
```bash
# 测试模式（随机动作）
cd /home/wang/Project/hil-serl
python examples/experiments/pick_ironroad/deploy.py --test_mode

# 或加载已训练模型
python examples/experiments/pick_ironroad/deploy.py --checkpoint_path=./checkpoints/your_model
```

### 快速启动
```bash
# 使用一键启动脚本
chmod +x examples/experiments/pick_ironroad/start_deployment.sh
./examples/experiments/pick_ironroad/start_deployment.sh
```

## 配置说明

### EnvConfig类（config.py）
- `ROBOT_NAME`: ROS2机械臂节点名称 (如 "robot_left")
- `GRIPPER_NAME`: ROS2夹爪节点名称 (如 "gripper_left")
- `TARGET_POSE`: 目标位姿（毫米和度）
- `RESET_POSE`: 重置位姿
- `ACTION_SCALE`: 动作缩放系数
- `ABS_POSE_LIMIT_*`: 安全边界
- `REALSENSE_CAMERAS`: 相机配置（复用franka_env相机系统）

### 坐标系转换
- **训练时格式**: 使用米和四元数 [x,y,z,qx,qy,qz,qw]
- **arm_control格式**: 使用毫米和欧拉角 [x,y,z,a,b,c,0]
- **自动转换**: IronroadEnv自动处理两种格式转换

## ROS2通信接口

### 机械臂控制：
- **订阅**: `/{robot_name}/joint_states`, `/{robot_name}/cartesian_states`
- **发布**: `/{robot_name}/control_command`
- **服务**: `/{robot_name}/control_service`

### 夹爪控制：
- **发布**: `/phantom1/state` (OmniState消息)
- **控制字段**: `msg.close_gripper` (True=关闭, False=打开)
- **硬件**: Robotiq 2F夹爪，通过Modbus串口

### 相机系统：
- **连接**: USB直连到RealSense相机
- **API**: Intel RealSense SDK (pyrealsense2)
- **识别**: 通过相机序列号区分多个设备

## 与原版USB Pick Up的差异

| 项目 | 原版(Franka) | 新版(Ironroad) |
|------|-------------|----------------|
| **控制方式** | HTTP + ROS1 | 直接ROS2 |
| **通信架构** | FrankaEnv → HTTP → franka_server → ROS1 | IronroadEnv → ROS2 → arm_control |
| **状态获取** | HTTP请求查询 | ROS2话题订阅 |
| **传感器** | 位置+速度+力+扭矩 | 仅位置（速度通过数值微分） |
| **夹爪控制** | ROS1夹爪话题 | **ROS2 OmniState话题** |
| **夹爪硬件** | Franka夹爪/Robotiq | **Robotiq 2F (Modbus串口)** |
| **相机系统** | franka_env.camera | **完全复用**franka_env.camera (USB) |
| **训练接口** | gym环境 | **完全相同**的gym环境接口 |

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