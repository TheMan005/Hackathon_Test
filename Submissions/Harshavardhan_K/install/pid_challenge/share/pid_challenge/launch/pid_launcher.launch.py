import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node  # This is correct!

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),

        LogInfo(
            condition=IfCondition(LaunchConfiguration('use_sim_time')),
            msg="Using simulation time"
        ),

        Node(
            package='pid_challenge',
            executable='pid_controller',
            name='pid_controller_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('/cmd_vel', '/cmd_vel')],
        ),
    ])
