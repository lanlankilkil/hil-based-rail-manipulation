#!/bin/bash
# Pick Ironroad Learner脚本

export XLA_PYTHON_CLIENT_PREALLOCATE=false
export XLA_PYTHON_CLIENT_MEM_FRACTION=.8
export PYTHONPATH="${PYTHONPATH}:$(pwd)"

python ../../train_rlpd.py "$@" \
    --exp_name=pick_ironroad \
    --checkpoint_path=../../experiments/pick_ironroad/debug \
    --demo_path=../../data/pick_ironroad_demos \
    --learner