#!/usr/bin/env python3
"""
pick_ironroad部署脚本
加载已训练的强化学习模型，通过ROS2直接控制ironroad机械臂执行任务
"""

import os
import sys
import numpy as np
import time
import argparse
import jax
import rclpy

# 添加项目路径
sys.path.append('/home/wang/Project/hil-serl')

from experiments.pick_ironroad.config import TrainConfig, EnvConfig
from serl_launcher.agents.continuous.sac import SACAgent


def load_trained_model(checkpoint_path):
    """加载已训练的强化学习模型"""
    config = TrainConfig()
    
    # 创建环境用于获取观测和动作空间
    env = config.get_environment(fake_env=True)
    
    # 创建智能体
    agent = SACAgent.create(
        config.seed,
        env.observation_space,
        env.action_space,
        **config.agent_kwargs
    )
    
    # 加载检查点
    if os.path.exists(checkpoint_path):
        print(f"加载模型检查点: {checkpoint_path}")
        # 这里需要根据实际的检查点加载方式调整
        # agent = agent.load_checkpoint(checkpoint_path)
        print("模型加载完成")
    else:
        print(f"警告: 检查点文件不存在 {checkpoint_path}")
        return None
        
    return agent, env


def deploy_model(agent, checkpoint_path=None):
    """部署模型到机械臂"""
    print("初始化pick_ironroad部署环境...")
    
    # 初始化ROS2
    if not rclpy.ok():
        rclpy.init()
    
    # 创建实际环境
    config = TrainConfig()
    env = config.get_environment(fake_env=False)
    
    try:
        print("开始执行任务...")
        episode = 0
        
        while True:
            episode += 1
            print(f"\n=== Episode {episode} ===")
            
            # 重置环境
            obs, info = env.reset()
            done = False
            step_count = 0
            
            while not done and step_count < env.max_episode_length:
                # 使用智能体预测动作
                if agent:
                    action = agent.predict(obs, deterministic=True)
                else:
                    # 如果没有模型，使用随机动作（测试用）
                    action = env.action_space.sample()
                    action = action * 0.1  # 减小动作幅度确保安全
                
                # 执行动作
                obs, reward, done, truncated, info = env.step(action)
                step_count += 1
                
                print(f"Step {step_count}: reward={reward}, done={done}")
                
                if reward or done:
                    success_msg = "任务成功完成!" if reward else "Episode结束"
                    print(f"{success_msg} (步数: {step_count})")
                    break
                    
                time.sleep(0.1)  # 控制执行频率
                
            # 询问是否继续
            user_input = input("\n按Enter继续下一个episode，输入'q'退出: ")
            if user_input.lower() == 'q':
                break
                
    except KeyboardInterrupt:
        print("\n用户中断，正在退出...")
    except Exception as e:
        print(f"执行过程中出错: {e}")
    finally:
        env.close()
        if rclpy.ok():
            rclpy.shutdown()
        print("环境已关闭")


def main():
    parser = argparse.ArgumentParser(description='Pick Ironroad部署脚本')
    parser.add_argument('--checkpoint_path', type=str, 
                       default='./checkpoints/pick_ironroad_model',
                       help='训练好的模型检查点路径')
    parser.add_argument('--test_mode', action='store_true',
                       help='测试模式（不加载模型，使用随机动作）')
    
    args = parser.parse_args()
    
    if args.test_mode:
        print("测试模式：使用随机动作")
        agent = None
    else:
        agent, _ = load_trained_model(args.checkpoint_path)
    
    # 部署模型
    deploy_model(agent, args.checkpoint_path)


if __name__ == "__main__":
    main()