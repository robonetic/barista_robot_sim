import os
import random

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.substitutions import Command


package_name = "barista_robot_description"

def generate_launch_description():
    urdf_file =  os.path.join(get_package_share_directory(
        package_name), "urdf", "barista_robot_model.urdf")

    rviz_file = os.path.join(get_package_share_directory(
        package_name), "rviz", "urdf_vis.rviz")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_node",
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', urdf_file])}],
        output="screen"
    )

    joint_state_pulbisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui_node",
        output="screen"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_node",
        output="screen",
        arguments=[{"-d", rviz_file}]
    )

    robot_name = "barista-bot"+"-"+str(int(random.random() * 10000))

    # Spawn ROBOT Set Gazebo
    spawn_barista_bot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', robot_name,
                   '-x', '0', '-y', '0', '-z', '0.2',
                   '-topic', '/robot_description'
        ]
    )


    return LaunchDescription(
        [robot_state_publisher_node, joint_state_pulbisher_node, rviz_node, spawn_barista_bot]
    )

