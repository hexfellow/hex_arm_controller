import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hex_arm_controller',
            executable='arm_controller',  # ROS 2 使用可执行文件，不使用 node 类型
            name='joint_helper',
            output='screen',
            remappings=[('/joint_cmd', '/xtopic_arm/joints_cmd')]  # 使用 remappings 进行话题重映射
        )
    ])
