#!/bin/bash
# 启动pick_ironroad学习脚本

# 设置环境变量
export PYTHONPATH="${PYTHONPATH}:$(pwd)"

echo "启动pick_ironroad强化学习训练..."
python examples/train_rlpd.py \
    --algo='rlpd' \
    --task='pick_ironroad' \
    --data_save_path='./data/pick_ironroad_training' \
    --demo_path='./data/pick_ironroad_demos' \
    --checkpoint_period=2000 \
    --eval_period=5000 \
    --max_episode_length=120

echo "训练完成"