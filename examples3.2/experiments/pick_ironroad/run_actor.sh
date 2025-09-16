#!/bin/bash
# 启动pick_ironroad演示录制脚本

# 设置环境变量
export PYTHONPATH="${PYTHONPATH}:$(pwd)"

# 启动ironroad HTTP服务器（后台运行）
echo "启动Ironroad HTTP服务器..."
python serl_robot_infra/robot_servers/ironroad_server.py \
    --robot_name=robot_left \
    --robot_ip=192.168.1.13 \
    --robot_port=6001 \
    --flask_port=5000 &

# 等待服务器启动
sleep 3

# 启动演示录制
echo "启动pick_ironroad演示录制..."
python examples/record_demos.py \
    --algo='bc' \
    --task='pick_ironroad' \
    --data_save_path='./data/pick_ironroad_demos' \
    --max_episode_length=120

echo "演示录制完成"