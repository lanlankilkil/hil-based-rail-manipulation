import os
import jax
import numpy as np
import jax.numpy as jnp

from serl_launcher.wrappers.serl_obs_wrappers import SERLObsWrapper
from serl_launcher.wrappers.chunking import ChunkingWrapper
from serl_launcher.networks.reward_classifier import load_classifier_func

from experiments.config import DefaultTrainingConfig
from experiments.pick_ironroad.wrapper import IronroadEnv, GripperPenaltyWrapper


class EnvConfig:
    # ROS2机械臂配置
    ROBOT_NAME: str = "robot_left"
    
    # 相机配置（可选，如果没有相机可以注释掉）
    REALSENSE_CAMERAS = {
        "wrist_1": {
            "serial_number": "127122270350", 
            "dim": (1280, 720),
            "exposure": 10500,
        },
        "side_policy": {
            "serial_number": "130322274175",
            "dim": (1280, 720), 
            "exposure": 13000,
        },
    }
    IMAGE_CROP = {
        "wrist_1": lambda img: img[50:-200, 200:-200],
        "side_policy": lambda img: img[250:500, 350:650],
    }
    
    # 任务位姿配置（毫米和度）
    # 需要根据实际的pick_ironroad任务场景调整
    TARGET_POSE = np.array([280.0, 0.0, 400.0, 0.0, 0.0, 0.0])  # [x,y,z,a,b,c]
    RESET_POSE = TARGET_POSE + np.array([0, 30, 50, 0, 0, 0])
    
    # 动作缩放参数
    ACTION_SCALE = np.array([15.0, 0.1, 1])  # [位置缩放(mm), 姿态缩放(rad), 夹爪缩放]
    
    # 随机重置配置
    RANDOM_RESET = True
    RANDOM_XY_RANGE = 10.0  # 毫米
    RANDOM_RZ_RANGE = 0.1   # 弧度
    
    # 安全边界限制（毫米和弧度）
    ABS_POSE_LIMIT_HIGH = TARGET_POSE + np.array([30, 60, 50, 0.1, 0.1, 0.3])
    ABS_POSE_LIMIT_LOW = TARGET_POSE - np.array([30, 10, 30, 0.1, 0.1, 0.3])
    
    # 控制参数
    MAX_EPISODE_LENGTH = 120
    DISPLAY_IMAGE = True


class TrainConfig(DefaultTrainingConfig):
    # 图像和状态键配置
    image_keys = ["side_policy", "wrist_1"] if hasattr(EnvConfig, 'REALSENSE_CAMERAS') else []
    classifier_keys = ["side_policy"] if hasattr(EnvConfig, 'REALSENSE_CAMERAS') else []
    proprio_keys = ["tcp_pose", "tcp_vel", "gripper_pose"]  # arm_control不提供力/扭矩传感器
    
    # 训练参数
    checkpoint_period = 2000
    cta_ratio = 2
    random_steps = 0
    discount = 0.98
    buffer_period = 1000
    encoder_type = "resnet-pretrained"
    setup_mode = "single-arm-learned-gripper"

    def get_environment(self, fake_env=False, save_video=False, classifier=False):
        env = IronroadEnv(
            fake_env=fake_env, 
            save_video=save_video, 
            config=EnvConfig()
        )
        
        # 包装器
        env = SERLObsWrapper(env, proprio_keys=self.proprio_keys)
        env = ChunkingWrapper(env, obs_horizon=1, act_exec_horizon=None)
        
        # 分类器（如果启用且有相机）
        if classifier and self.classifier_keys:
            try:
                classifier_func = load_classifier_func(
                    key=jax.random.PRNGKey(0),
                    sample=env.observation_space.sample(),
                    image_keys=self.classifier_keys,
                    checkpoint_path=os.path.abspath("classifier_ckpt/"),
                )

                def reward_func(obs):
                    sigmoid = lambda x: 1 / (1 + jnp.exp(-x))
                    return int(sigmoid(classifier_func(obs)) > 0.7 and obs["state"][0, 0] > 0.4)

                # 如果需要使用分类器，可以在这里添加包装器
                # env = MultiCameraBinaryRewardClassifierWrapper(env, reward_func)
            except Exception as e:
                print(f"警告：无法加载分类器: {e}")
        
        # 夹爪惩罚
        env = GripperPenaltyWrapper(env, penalty=-0.02)
        return env