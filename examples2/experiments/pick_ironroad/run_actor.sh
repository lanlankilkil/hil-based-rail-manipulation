#!/bin/bash
# Pick Ironroad Actor脚本

export XLA_PYTHON_CLIENT_PREALLOCATE=false
export XLA_PYTHON_CLIENT_MEM_FRACTION=.1
export PYTHONPATH="${PYTHONPATH}:$(pwd)"

# 启动arm_control
ros2 launch arm_control left_arm_control.launch.py &
ARM_PID=$!

sleep 3

# 评估模式使用deploy.py，训练模式使用train_rlpd.py
if [[ "$*" == *"--eval"* ]]; then
    python ../../experiments/pick_ironroad/deploy.py "$@"
else
    python ../../train_rlpd.py "$@" \
        --exp_name=pick_ironroad \
        --checkpoint_path=../../experiments/pick_ironroad/debug \
        --actor
fi

# 清理
kill $ARM_PID 2>/dev/null || true