#!/usr/bin/env python3

import numpy as np
import gymnasium as gym
import threading
import time
from typing import Optional, Dict, Any
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class TeleopCommandMsg:
    """Custom message structure for teleop commands"""
    def __init__(self):
        self.linear = [0.0, 0.0, 0.0]  # x, y, z
        self.angular = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        self.gripper = [0.0, 0.0]  # left_gripper, right_gripper
        self.buttons = [0, 0, 0, 0]  # button states
        self.timestamp = time.time()
        self.active = False  # whether teleop is active


class ROS2TeleopNode(Node):
    """ROS2 node for receiving teleop commands"""
    
    def __init__(self, node_name='teleop_intervention_node'):
        super().__init__(node_name)
        
        # QoS profile for real-time communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Latest teleop command
        self.latest_command = TeleopCommandMsg()
        self.command_lock = threading.Lock()
        
        # Subscribers for different teleop inputs
        self.twist_sub = self.create_subscription(
            Twist, 
            '/teleop/cmd_vel', 
            self.twist_callback, 
            qos_profile
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/teleop/target_pose',
            self.pose_callback,
            qos_profile
        )
        
        self.gripper_sub = self.create_subscription(
            Float32MultiArray,
            '/teleop/gripper_cmd',
            self.gripper_callback,
            qos_profile
        )
        
        self.button_sub = self.create_subscription(
            Float32MultiArray,
            '/teleop/buttons',
            self.button_callback,
            qos_profile
        )
        
        self.active_sub = self.create_subscription(
            Bool,
            '/teleop/active',
            self.active_callback,
            qos_profile
        )
        
        # Joint state subscriber (for monitoring current robot state)
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            qos_profile
        )
        
        self.current_joint_state = None
        self.get_logger().info(f'ROS2 Teleop Node initialized: {node_name}')
    
    def twist_callback(self, msg: Twist):
        """Handle Twist commands (velocity control)"""
        with self.command_lock:
            self.latest_command.linear = [msg.linear.x, msg.linear.y, msg.linear.z]
            self.latest_command.angular = [msg.angular.x, msg.angular.y, msg.angular.z]
            self.latest_command.timestamp = time.time()
    
    def pose_callback(self, msg: PoseStamped):
        """Handle pose commands (position control)"""
        with self.command_lock:
            # Convert pose to relative movement
            # This is a simplified conversion - you may need more sophisticated logic
            self.latest_command.linear = [
                msg.pose.position.x * 0.1,  # scale factor
                msg.pose.position.y * 0.1,
                msg.pose.position.z * 0.1
            ]
            self.latest_command.timestamp = time.time()
    
    def gripper_callback(self, msg: Float32MultiArray):
        """Handle gripper commands"""
        with self.command_lock:
            if len(msg.data) >= 2:
                self.latest_command.gripper = list(msg.data[:2])
            self.latest_command.timestamp = time.time()
    
    def button_callback(self, msg: Float32MultiArray):
        """Handle button states"""
        with self.command_lock:
            if len(msg.data) >= 4:
                self.latest_command.buttons = [int(x) for x in msg.data[:4]]
            self.latest_command.timestamp = time.time()
    
    def active_callback(self, msg: Bool):
        """Handle teleop active/inactive state"""
        with self.command_lock:
            self.latest_command.active = msg.data
            self.latest_command.timestamp = time.time()
    
    def joint_callback(self, msg: JointState):
        """Handle current joint states"""
        self.current_joint_state = msg
    
    def get_latest_command(self) -> TeleopCommandMsg:
        """Get the latest teleop command safely"""
        with self.command_lock:
            return self.latest_command


class ROS2TeleopIntervention(gym.ActionWrapper):
    """
    Gym wrapper that provides human intervention through ROS2 teleop commands
    """
    
    def __init__(self, env, action_indices=None, gripper_enabled=True, 
                 timeout_threshold=0.5, intervention_threshold=0.001,
                 node_name='teleop_intervention'):
        super().__init__(env)
        
        self.gripper_enabled = gripper_enabled
        self.action_indices = action_indices
        self.timeout_threshold = timeout_threshold  # seconds
        self.intervention_threshold = intervention_threshold
        
        # Initialize ROS2
        if not rclpy.ok():
            rclpy.init()
        
        # Create ROS2 node
        self.ros_node = ROS2TeleopNode(node_name)
        
        # Start ROS2 spinning in a separate thread
        self.ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self.ros_thread.start()
        
        # Button states for gripper control
        self.left1, self.left2, self.right1, self.right2 = False, False, False, False
        
        print(f"ROS2 Teleop Intervention initialized with node: {node_name}")
    
    def _spin_ros(self):
        """Spin ROS2 node in separate thread"""
        try:
            rclpy.spin(self.ros_node)
        except Exception as e:
            print(f"ROS2 spinning error: {e}")
    
    def action(self, action: np.ndarray) -> tuple:
        """
        Input:
        - action: policy action
        Output:
        - action: teleop action if available and valid; else, policy action
        - intervened: boolean indicating if intervention occurred
        """
        intervened = False
        
        # Get latest teleop command
        teleop_cmd = self.ros_node.get_latest_command()
        
        # Check if teleop is active and not timed out
        current_time = time.time()
        is_teleop_active = (
            teleop_cmd.active and 
            (current_time - teleop_cmd.timestamp) < self.timeout_threshold
        )
        
        if not is_teleop_active:
            return action, False
        
        # Convert teleop command to robot action
        expert_action = self._convert_teleop_to_action(teleop_cmd)
        
        # Update button states
        if len(teleop_cmd.buttons) >= 4:
            self.left1, self.left2, self.right1, self.right2 = teleop_cmd.buttons
        
        # Handle gripper actions if enabled
        if self.gripper_enabled:
            left_gripper_action, right_gripper_action = self._process_gripper_commands(teleop_cmd)
            
            # Combine with movement commands
            if len(expert_action) == 12:  # dual arm case
                expert_action[6] = left_gripper_action  # left gripper
                expert_action[13] = right_gripper_action  # right gripper
            elif len(expert_action) == 7:  # single arm case
                expert_action[6] = left_gripper_action
        
        # Apply action indices filter if specified
        if self.action_indices is not None:
            filtered_expert_action = np.zeros_like(expert_action)
            filtered_expert_action[self.action_indices] = expert_action[self.action_indices]
            expert_action = filtered_expert_action
        
        # Check if intervention should occur
        if np.linalg.norm(expert_action) > self.intervention_threshold:
            intervened = True
            return expert_action, True
        
        return action, False
    
    def _convert_teleop_to_action(self, teleop_cmd: TeleopCommandMsg) -> np.ndarray:
        """Convert teleop command to robot action format"""
        # Basic conversion - you may need to adjust based on your robot's action space
        action = np.zeros(14)  # assuming dual arm setup (7 DOF each)
        
        # Left arm (first 6 DOF: position + orientation)
        action[0] = teleop_cmd.linear[0]   # x
        action[1] = teleop_cmd.linear[1]   # y  
        action[2] = teleop_cmd.linear[2]   # z
        action[3] = teleop_cmd.angular[0]  # roll
        action[4] = teleop_cmd.angular[1]  # pitch
        action[5] = teleop_cmd.angular[2]  # yaw
        # action[6] will be set by gripper processing
        
        # Right arm (next 6 DOF: position + orientation)
        action[7] = teleop_cmd.linear[0]   # x (mirror or separate control)
        action[8] = teleop_cmd.linear[1]   # y
        action[9] = teleop_cmd.linear[2]   # z
        action[10] = teleop_cmd.angular[0] # roll
        action[11] = teleop_cmd.angular[1] # pitch
        action[12] = teleop_cmd.angular[2] # yaw
        # action[13] will be set by gripper processing
        
        return action
    
    def _process_gripper_commands(self, teleop_cmd: TeleopCommandMsg) -> tuple:
        """Process gripper commands from teleop"""
        left_gripper_action = 0.0
        right_gripper_action = 0.0
        
        # From direct gripper commands
        if len(teleop_cmd.gripper) >= 2:
            left_gripper_action = teleop_cmd.gripper[0]
            right_gripper_action = teleop_cmd.gripper[1]
        
        # From button commands (override direct commands)
        if self.left1:  # close left gripper
            left_gripper_action = np.random.uniform(-1, -0.9)
        elif self.left2:  # open left gripper
            left_gripper_action = np.random.uniform(0.9, 1)
        
        if self.right1:  # close right gripper
            right_gripper_action = np.random.uniform(-1, -0.9)
        elif self.right2:  # open right gripper
            right_gripper_action = np.random.uniform(0.9, 1)
        
        return left_gripper_action, right_gripper_action
    
    def step(self, action):
        """Environment step with ROS2 teleop intervention"""
        new_action, replaced = self.action(action)
        
        obs, rew, done, truncated, info = self.env.step(new_action)
        
        if replaced:
            info["intervene_action"] = new_action
        
        # Add teleop-specific info
        teleop_cmd = self.ros_node.get_latest_command()
        info["teleop_active"] = teleop_cmd.active
        info["teleop_timestamp"] = teleop_cmd.timestamp
        info["left1"] = self.left1
        info["left2"] = self.left2
        info["right1"] = self.right1
        info["right2"] = self.right2
        
        return obs, rew, done, truncated, info
    
    def reset(self, **kwargs):
        """Reset environment"""
        return self.env.reset(**kwargs)
    
    def close(self):
        """Clean up ROS2 resources"""
        if hasattr(self, 'ros_node'):
            self.ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        super().close()


class DualROS2TeleopIntervention(ROS2TeleopIntervention):
    """
    Specialized version for dual arm control with separate teleop sources
    """
    
    def __init__(self, env, **kwargs):
        super().__init__(env, **kwargs)
        
        # Additional subscribers for right arm
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        self.right_twist_sub = self.ros_node.create_subscription(
            Twist,
            '/teleop/right/cmd_vel',
            self.right_twist_callback,
            qos_profile
        )
        
        self.right_pose_sub = self.ros_node.create_subscription(
            PoseStamped,
            '/teleop/right/target_pose', 
            self.right_pose_callback,
            qos_profile
        )
        
        # Separate command storage for right arm
        self.right_command = TeleopCommandMsg()
    
    def right_twist_callback(self, msg: Twist):
        """Handle right arm twist commands"""
        with self.ros_node.command_lock:
            self.right_command.linear = [msg.linear.x, msg.linear.y, msg.linear.z]
            self.right_command.angular = [msg.angular.x, msg.angular.y, msg.angular.z]
            self.right_command.timestamp = time.time()
    
    def right_pose_callback(self, msg: PoseStamped):
        """Handle right arm pose commands"""
        with self.ros_node.command_lock:
            self.right_command.linear = [
                msg.pose.position.x * 0.1,
                msg.pose.position.y * 0.1,
                msg.pose.position.z * 0.1
            ]
            self.right_command.timestamp = time.time()
    
    def _convert_teleop_to_action(self, teleop_cmd: TeleopCommandMsg) -> np.ndarray:
        """Convert dual teleop commands to robot action format"""
        action = np.zeros(14)  # dual arm setup
        
        # Left arm from main teleop command
        action[0:6] = teleop_cmd.linear + teleop_cmd.angular
        
        # Right arm from separate teleop command
        current_time = time.time()
        if (current_time - self.right_command.timestamp) < self.timeout_threshold:
            action[7:13] = self.right_command.linear + self.right_command.angular
        
        return action
