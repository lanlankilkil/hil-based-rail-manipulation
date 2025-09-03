from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    Nodes = []
    Nodes.append(
        Node(
            package='arm_control',
            executable='robot_arm_node',
            name='robot_left_arm',
            parameters=[
                {'robot_name': 'robot_left'},
                {'robot_ip': '192.168.1.13'},
                {'robot_port': 6001},
                {'omni_command_topic': '/phantom1/state'},
                {'omni_button_event_topic': '/phantom1/button'}
            ]
        ))
    Nodes.append(Node(
            package='arm_control',
            executable='gripper',
            name='gripper_left',
            parameters=[
                {'device_path': '/dev/gripper_left'},
                {'subscribe_topic': '/phantom1/state'},
            ]
        ))
    return LaunchDescription(
        Nodes
    )    