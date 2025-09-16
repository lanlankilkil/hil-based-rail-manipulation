#!/bin/bash
# pick_ironroad ROS2直接部署启动脚本

echo "=== Pick Ironroad ROS2 部署启动脚本 ==="

# 1. 启动arm_control ROS2节点
echo "1. 启动arm_control ROS2节点..."
ros2 launch arm_control left_arm_control.launch.py &
ARM_CONTROL_PID=$!

# 等待ROS2节点启动
echo "等待arm_control节点启动..."
sleep 5

# 2. 直接部署推理模型（不需要HTTP服务器）
echo "2. 开始部署pick_ironroad模型..."
cd /home/wang/Project/hil-serl

echo "可选项："
echo "  a) 测试模式（随机动作）: python examples/experiments/pick_ironroad/deploy.py --test_mode"
echo "  b) 加载模型部署: python examples/experiments/pick_ironroad/deploy.py --checkpoint_path=路径"

echo ""
echo "启动完成！服务进程："
echo "  - arm_control ROS2: PID $ARM_CONTROL_PID"
echo ""
echo "要停止服务，请运行: kill $ARM_CONTROL_PID"

# 保持脚本运行
wait