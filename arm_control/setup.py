from setuptools import setup
import os
from glob import glob

package_name = 'arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # 对应arm_control/子目录
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),  # 包含msg文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lz',
    maintainer_email='lz@example.com',
    description='Arm control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    # entry_points={
    #     'console_scripts': [
    #         'robot_arm_node = arm_control.robot_arm_node:main',
    #     ],
    # },
)
