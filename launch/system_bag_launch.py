import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    ros_bag = LaunchConfiguration('ros_bag')

    ros_bag_arg = DeclareLaunchArgument(
        'ros_bag',
        default_value='False'
    )
    talker_node = Node(
        package='beginner_tutorials',
        executable='talker'
    )
    listener_node = Node(
        package='beginner_tutorials',
        executable='listener'
    )
    server_node = Node(
        package='beginner_tutorials',
        executable='server'
    )
    ros_bag_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                ros_bag,
                ' == True'
            ])
        ),
        cmd=[[
            'ros2 run beginner_tutorials simple_bag_recorder'
        ]],
        shell=True
    )

    return LaunchDescription([
        ros_bag_arg,
        talker_node,
        listener_node,
        server_node,
        TimerAction(
            period=2.0,
            actions=[ros_bag_conditioned],
        )
    ])