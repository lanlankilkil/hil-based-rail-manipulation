# Pick Ironroad 部署指南

## 概述
pick_ironroad是基于USB Pick Up and Insertion改造的新机械臂部署方案，使用arm_control（ROS2）替代原有的Franka（ROS1）控制系统。采用直接ROS2通信方式，无需HTTP中间层。

## 系统架构

```
已训练的强化学习模型 
    ↓ (预测动作)
IronroadEnv (gym接口)
    ↓ (ROS2消息)
IronroadROS2Node 
    ↓ (ROS2话题/服务)
arm_control节点 (ROS2)
    ↓ (SDK调用)
机械臂硬件
```

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

### 1. 启动arm_control节点
```bash
# 在终端1中启动
ros2 launch arm_control left_arm_control.launch.py
```

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
- `ROBOT_NAME`: ROS2机械臂节点名称
- `TARGET_POSE`: 目标位姿（毫米和度）
- `RESET_POSE`: 重置位姿
- `ACTION_SCALE`: 动作缩放系数
- `ABS_POSE_LIMIT_*`: 安全边界
- `REALSENSE_CAMERAS`: 相机配置（复用franka_env相机系统）

### 坐标系转换
- **训练时格式**: 使用米和四元数 [x,y,z,qx,qy,qz,qw]
- **arm_control格式**: 使用毫米和欧拉角 [x,y,z,a,b,c,0]
- **自动转换**: IronroadEnv自动处理两种格式转换

## 与原版USB Pick Up的差异

| 项目 | 原版(Franka) | 新版(Ironroad) |
|------|-------------|----------------|
| **控制方式** | HTTP + ROS1 | 直接ROS2 |
| **通信架构** | FrankaEnv → HTTP → franka_server → ROS1 | IronroadEnv → ROS2 → arm_control |
| **状态获取** | HTTP请求查询 | ROS2话题订阅 |
| **传感器** | 位置+速度+力+扭矩 | 仅位置（速度通过数值微分） |
| **相机系统** | franka_env.camera | **完全复用**franka_env.camera |
| **训练接口** | gym环境 | **完全相同**的gym环境接口 |

## ROS2通信接口

### 订阅的话题：
- `/{robot_name}/joint_states` - 关节状态
- `/{robot_name}/cartesian_states` - 直角坐标状态

### 发布的话题：
- `/{robot_name}/control_command` - 位姿控制命令

### 服务：
- `/{robot_name}/control_service` - 错误清除等命令

## 相机系统
- **完全兼容**：可以直接使用原有的RealSense相机配置
- **序列号配置**：在`EnvConfig.REALSENSE_CAMERAS`中设置
- **图像处理**：复用原有的裁剪和缩放逻辑

## 故障排除

1. **ROS2节点无法连接机械臂**: 检查`robot_ip`和`robot_port`配置
2. **状态订阅失败**: 确认arm_control节点正常运行
3. **模型加载失败**: 检查检查点文件路径和格式
4. **动作执行异常**: 检查安全边界和动作缩放参数
5. **相机问题**: 检查RealSense驱动和序列号配置

## 优势
- **更低延迟**: 直接ROS2通信，无HTTP转换
- **更高实时性**: 状态自动推送，非查询式
- **架构简化**: 减少中间层
- **相机复用**: 无需重新实现相机功能