#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Float32MultiArray, Header
from sensor_msgs.msg import Joy
import time
import math
import numpy as np


class TeleopPublisher(Node):
    """
    Example teleop publisher that converts joystick input to robot commands
    You can replace this with your actual teleop device interface
    """
    
    def __init__(self):
        super().__init__('teleop_publisher')
        
        # Publishers for teleop commands
        self.twist_pub = self.create_publisher(Twist, '/teleop/cmd_vel', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/teleop/target_pose', 10)
        self.gripper_pub = self.create_publisher(Float32MultiArray, '/teleop/gripper_cmd', 10)
        self.button_pub = self.create_publisher(Float32MultiArray, '/teleop/buttons', 10)
        self.active_pub = self.create_publisher(Bool, '/teleop/active', 10)
        
        # Publishers for dual arm (right arm)
        self.right_twist_pub = self.create_publisher(Twist, '/teleop/right/cmd_vel', 10)
        self.right_pose_pub = self.create_publisher(PoseStamped, '/teleop/right/target_pose', 10)
        
        # Subscriber for joystick input (optional)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Timer for publishing at regular intervals
        self.timer = self.create_timer(0.02, self.publish_teleop_commands)  # 50Hz
        
        # Internal state
        self.teleop_active = False
        self.left_arm_cmd = Twist()
        self.right_arm_cmd = Twist()
        self.gripper_cmd = [0.0, 0.0]  # [left, right]
        self.button_states = [0, 0, 0, 0]
        
        # For demo purposes - simulate sinusoidal motion
        self.demo_time = 0.0
        self.demo_mode = False
        
        self.get_logger().info('Teleop Publisher Node started')
    
    def joy_callback(self, msg: Joy):
        """
        Handle joystick input and convert to robot commands
        Modify this based on your specific joystick/controller layout
        """
        if len(msg.axes) < 4 or len(msg.buttons) < 4:
            return
        
        # Check if teleop is activated (e.g., by holding a specific button)
        self.teleop_active = msg.buttons[0] == 1  # Button 0 activates teleop
        
        if not self.teleop_active:
            return
        
        # Map joystick axes to robot motion
        # Left stick: X-Y translation
        self.left_arm_cmd.linear.x = msg.axes[0] * 0.1  # Scale factor
        self.left_arm_cmd.linear.y = msg.axes[1] * 0.1
        
        # Right stick: Z translation and yaw rotation
        self.left_arm_cmd.linear.z = msg.axes[3] * 0.1
        self.left_arm_cmd.angular.z = msg.axes[2] * 0.2
        
        # Trigger buttons for gripper control
        if len(msg.axes) >= 6:
            # Left trigger: left gripper, Right trigger: right gripper
            self.gripper_cmd[0] = (msg.axes[4] - 1.0) / -2.0  # Convert from [-1,1] to [0,1]
            self.gripper_cmd[1] = (msg.axes[5] - 1.0) / -2.0
        
        # Button states
        self.button_states = list(msg.buttons[:4])
        
        # For dual arm, you might use additional axes or buttons
        if len(msg.axes) >= 8:
            self.right_arm_cmd.linear.x = msg.axes[6] * 0.1
            self.right_arm_cmd.linear.y = msg.axes[7] * 0.1
    
    def enable_demo_mode(self):
        """Enable demo mode with sinusoidal motion"""
        self.demo_mode = True
        self.teleop_active = True
        self.get_logger().info('Demo mode enabled')
    
    def disable_demo_mode(self):
        """Disable demo mode"""
        self.demo_mode = False
        self.teleop_active = False
        self.get_logger().info('Demo mode disabled')
    
    def publish_teleop_commands(self):
        """Publish teleop commands at regular intervals"""
        current_time = self.get_clock().now()
        
        # Demo mode: generate sinusoidal motion
        if self.demo_mode:
            self.demo_time += 0.02  # 50Hz
            
            # Generate smooth sinusoidal motion
            self.left_arm_cmd.linear.x = 0.05 * math.sin(self.demo_time)
            self.left_arm_cmd.linear.y = 0.03 * math.cos(self.demo_time * 1.5)
            self.left_arm_cmd.linear.z = 0.02 * math.sin(self.demo_time * 0.5)
            self.left_arm_cmd.angular.z = 0.1 * math.sin(self.demo_time * 0.8)
            
            # Gripper demo
            self.gripper_cmd[0] = 0.5 + 0.5 * math.sin(self.demo_time * 2)
            self.gripper_cmd[1] = 0.5 + 0.5 * math.cos(self.demo_time * 2)
            
            # Button demo
            self.button_states[0] = 1 if math.sin(self.demo_time * 3) > 0 else 0
            self.button_states[1] = 1 if math.cos(self.demo_time * 3) > 0 else 0
        
        # Publish active state
        active_msg = Bool()
        active_msg.data = self.teleop_active
        self.active_pub.publish(active_msg)
        
        if not self.teleop_active:
            return
        
        # Publish twist commands
        self.left_arm_cmd.header = Header()
        self.left_arm_cmd.header.stamp = current_time.to_msg()
        self.twist_pub.publish(self.left_arm_cmd)
        
        # Publish right arm commands
        self.right_arm_cmd.header = Header()
        self.right_arm_cmd.header.stamp = current_time.to_msg()
        self.right_twist_pub.publish(self.right_arm_cmd)
        
        # Publish pose commands (alternative to twist)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time.to_msg()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x = self.left_arm_cmd.linear.x
        pose_msg.pose.position.y = self.left_arm_cmd.linear.y
        pose_msg.pose.position.z = self.left_arm_cmd.linear.z
        # Note: You would typically set proper orientation quaternion
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)
        
        # Publish gripper commands
        gripper_msg = Float32MultiArray()
        gripper_msg.data = self.gripper_cmd
        self.gripper_pub.publish(gripper_msg)
        
        # Publish button states
        button_msg = Float32MultiArray()
        button_msg.data = [float(x) for x in self.button_states]
        self.button_pub.publish(button_msg)


def main(args=None):
    rclpy.init(args=args)
    
    teleop_publisher = TeleopPublisher()
    
    # For demo purposes, enable demo mode after 2 seconds
    def enable_demo():
        time.sleep(2)
        teleop_publisher.enable_demo_mode()
    
    import threading
    demo_thread = threading.Thread(target=enable_demo, daemon=True)
    demo_thread.start()
    
    try:
        rclpy.spin(teleop_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
