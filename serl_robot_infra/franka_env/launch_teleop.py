#!/usr/bin/env python3

"""
Launch script for ROS2 teleop intervention system
This script demonstrates how to set up and use the ROS2 teleop system
"""

import rclpy
import subprocess
import time
import threading
import os
import signal
import sys
from serl_robot_infra.franka_env.teleop_publisher import TeleopPublisher


class ROS2TeleopLauncher:
    """
    Launcher for ROS2 teleop system components
    """
    
    def __init__(self):
        self.processes = []
        self.nodes = []
        
    def launch_teleop_publisher(self):
        """Launch the teleop publisher node"""
        print("Starting teleop publisher...")
        if not rclpy.ok():
            rclpy.init()
        
        teleop_node = TeleopPublisher()
        self.nodes.append(teleop_node)
        
        # Enable demo mode for testing
        def enable_demo():
            time.sleep(2)
            teleop_node.enable_demo_mode()
            print("Demo mode enabled - robot should start receiving teleop commands")
        
        demo_thread = threading.Thread(target=enable_demo, daemon=True)
        demo_thread.start()
        
        return teleop_node
    
    def launch_joy_node(self):
        """Launch ROS2 joy node for joystick input (optional)"""
        try:
            print("Starting joy node...")
            joy_process = subprocess.Popen([
                'ros2', 'run', 'joy', 'joy_node',
                '--ros-args', '-r', '__ns:=/teleop'
            ])
            self.processes.append(joy_process)
            return joy_process
        except FileNotFoundError:
            print("Warning: joy node not found. Install ros-humble-joy if you want joystick support")
            return None
    
    def launch_rviz(self):
        """Launch RViz for visualization (optional)"""
        try:
            print("Starting RViz...")
            rviz_process = subprocess.Popen([
                'ros2', 'run', 'rviz2', 'rviz2',
                '-d', 'teleop_config.rviz'
            ])
            self.processes.append(rviz_process)
            return rviz_process
        except FileNotFoundError:
            print("Warning: RViz not found. Install ros-humble-rviz2 if you want visualization")
            return None
    
    def shutdown(self):
        """Clean shutdown of all components"""
        print("Shutting down teleop system...")
        
        # Destroy ROS2 nodes
        for node in self.nodes:
            try:
                node.destroy_node()
            except:
                pass
        
        # Terminate processes
        for process in self.processes:
            try:
                process.terminate()
                process.wait(timeout=5)
            except:
                try:
                    process.kill()
                except:
                    pass
        
        if rclpy.ok():
            rclpy.shutdown()
        
        print("Teleop system shutdown complete")


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\nReceived interrupt signal...")
    launcher.shutdown()
    sys.exit(0)


def main():
    global launcher
    
    print("="*60)
    print("ROS2 Teleop Intervention System Launcher")
    print("="*60)
    
    launcher = ROS2TeleopLauncher()
    
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Launch components
        teleop_node = launcher.launch_teleop_publisher()
        
        # Optional: launch joystick support
        joy_process = launcher.launch_joy_node()
        
        # Optional: launch RViz
        # rviz_process = launcher.launch_rviz()
        
        print("\nTeleop system is running...")
        print("The system will publish demo commands automatically.")
        print("In your robot training script, use intervention_mode='ros2'")
        print("\nPress Ctrl+C to shutdown")
        print("-"*60)
        
        # Keep the main thread alive
        rclpy.spin(teleop_node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        launcher.shutdown()


if __name__ == '__main__':
    main()
