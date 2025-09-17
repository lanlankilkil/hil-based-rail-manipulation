# ROS2 Teleop Intervention System

这个系统允许通过ROS2 topics接收遥操作命令来进行人工干预，替代或补充原有的SpaceMouse干预机制。

## 系统架构

```
遥操作设备 -> ROS2 Topics -> ROS2TeleopIntervention -> 机器人环境
     |                              |
  操纵杆/键盘              干预动作生成与过滤
  手机应用
  VR控制器
```

## 主要组件

### 1. ROS2TeleopIntervention (ros2_teleop_intervention.py)
- 主要的干预包装器，替代SpacemouseIntervention
- 监听ROS2 topics获取遥操作命令
- 将遥操作命令转换为机器人动作
- 在环境的info中提供`intervene_action`

### 2. TeleopPublisher (teleop_publisher.py)
- 示例遥操作命令发布器
- 可以接收操纵杆输入或生成演示命令
- 发布标准化的遥操作消息

### 3. 环境配置更新
- 支持不同的干预模式：spacemouse, ros2, both, none
- 在get_environment()方法中指定intervention_mode参数

## ROS2 Topics接口

### 输入Topics (机器人订阅)

#### 基本控制
- `/teleop/cmd_vel` (geometry_msgs/Twist): 速度控制命令
- `/teleop/target_pose` (geometry_msgs/PoseStamped): 位置控制命令
- `/teleop/gripper_cmd` (std_msgs/Float32MultiArray): 夹爪控制 [left, right]
- `/teleop/buttons` (std_msgs/Float32MultiArray): 按钮状态 [btn0, btn1, btn2, btn3]
- `/teleop/active` (std_msgs/Bool): 遥操作激活状态

#### 双臂控制 (可选)
- `/teleop/right/cmd_vel` (geometry_msgs/Twist): 右臂速度控制
- `/teleop/right/target_pose` (geometry_msgs/PoseStamped): 右臂位置控制

#### 监控Topics
- `/joint_states` (sensor_msgs/JointState): 当前关节状态

### 输出Topics (可选，用于反馈)

可以添加以下topics来提供反馈：
- `/teleop/status` (std_msgs/String): 系统状态
- `/teleop/force_feedback` (geometry_msgs/WrenchStamped): 力反馈

## 使用方法

### 1. 基本使用

```python
# 在你的训练脚本中
python record_demos.py --exp_name=object_handover --intervention_mode=ros2

# 或者在配置中
env = config.get_environment(
    fake_env=False, 
    save_video=False, 
    classifier=True,
    intervention_mode="ros2"  # 使用ROS2干预
)
```

### 2. 启动遥操作系统

```bash
# 终端1: 启动遥操作发布器
python serl_robot_infra/franka_env/launch_teleop.py

# 终端2: 运行机器人训练/演示
python record_demos.py --exp_name=object_handover --intervention_mode=ros2
```

### 3. 混合模式使用

```python
# 同时使用SpaceMouse和ROS2干预
env = config.get_environment(intervention_mode="both")
```

## 自定义遥操作设备集成

### 1. 操纵杆/手柄

```python
# 安装joy包
sudo apt install ros-humble-joy

# 启动joy节点
ros2 run joy joy_node

# 修改teleop_publisher.py中的joy_callback()来适配你的手柄
```

### 2. 键盘控制

```python
# 创建键盘输入节点
class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        # 实现键盘监听和命令发布
```

### 3. 手机应用

```python
# 使用rosbridge_server为手机应用提供WebSocket接口
sudo apt install ros-humble-rosbridge-server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 4. VR控制器

```python
# 集成VR SDK，将VR控制器数据转换为ROS2消息
# 例如：Oculus, HTC Vive, 或其他VR设备
```

## 参数配置

### 干预检测参数

```python
intervention_threshold = 0.001  # 最小干预阈值
timeout_threshold = 0.5         # 命令超时时间(秒)
```

### 动作映射

```python
# 在_convert_teleop_to_action()中自定义动作映射
def _convert_teleop_to_action(self, teleop_cmd):
    # 自定义遥操作命令到机器人动作的映射
    action[0] = teleop_cmd.linear[0] * scale_factor
    # ...
```

## 扩展功能

### 1. 力反馈

```python
# 添加力反馈支持
class ForcefeedbackTeleop(ROS2TeleopIntervention):
    def __init__(self, env, **kwargs):
        super().__init__(env, **kwargs)
        self.force_pub = self.ros_node.create_publisher(
            WrenchStamped, '/teleop/force_feedback', 10
        )
    
    def step(self, action):
        obs, rew, done, truncated, info = super().step(action)
        # 发布力反馈信息
        self.publish_force_feedback(info)
        return obs, rew, done, truncated, info
```

### 2. 多用户协作

```python
# 支持多个操作员同时控制
class MultiUserTeleop(ROS2TeleopIntervention):
    def __init__(self, env, **kwargs):
        super().__init__(env, **kwargs)
        # 添加多用户命令融合逻辑
```

### 3. 安全监控

```python
# 添加安全检查
def check_safety_limits(self, action):
    # 检查动作是否在安全范围内
    if np.any(np.abs(action) > self.safety_limits):
        return False
    return True
```

## 调试和监控

### 1. 查看topics

```bash
# 列出所有teleop相关topics
ros2 topic list | grep teleop

# 监控特定topic
ros2 topic echo /teleop/cmd_vel
ros2 topic echo /teleop/active
```

### 2. 日志记录

```python
# 在ROS2TeleopIntervention中添加详细日志
self.get_logger().info(f"Intervention: {intervened}, Command: {teleop_cmd.linear}")
```

### 3. 可视化

```bash
# 使用RViz查看机器人状态和目标
ros2 run rviz2 rviz2

# 使用rqt_graph查看node连接
ros2 run rqt_graph rqt_graph
```

## 故障排除

### 常见问题

1. **ROS2节点无法启动**
   - 检查ROS2环境是否正确设置
   - 确保rclpy已安装

2. **遥操作命令不响应**
   - 检查`/teleop/active`是否为true
   - 验证命令是否超过`intervention_threshold`

3. **夹爪控制不工作**
   - 确认`gripper_enabled=True`
   - 检查夹爪命令格式

4. **延迟过高**
   - 调整QoS设置
   - 减少发布频率
   - 检查网络连接

### 性能优化

1. **减少延迟**
   ```python
   # 使用BEST_EFFORT QoS
   qos_profile = QoSProfile(
       reliability=ReliabilityPolicy.BEST_EFFORT,
       depth=1
   )
   ```

2. **减少CPU使用**
   ```python
   # 降低发布频率
   self.timer = self.create_timer(0.05, self.publish_commands)  # 20Hz instead of 50Hz
   ```

## 总结

这个ROS2遥操作干预系统提供了灵活的人工干预机制，可以轻松集成各种遥操作设备。通过标准化的ROS2接口，你可以：

1. 使用任何支持ROS2的设备进行遥操作
2. 实现多模态干预（同时使用多种输入设备）
3. 添加自定义的安全检查和力反馈
4. 支持远程遥操作和多用户协作

系统的模块化设计使得添加新的遥操作设备或修改控制逻辑变得简单直接。
