import os
import random
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

package_name = "barista_robot_description"

def generate_launch_description():
    rviz_file = os.path.join(get_package_share_directory(
        package_name), "rviz", "xacro_vis.rviz")

    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    robot_model_path = os.path.join(
        get_package_share_directory(package_name))

    xacro_file = os.path.join(robot_model_path, 'xacro', 'barista_robot_model.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_node",
        parameters=[params],
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

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
        )
    )

    robot_name = "barista-bot"+"-"+str(int(random.random() * 10000))

    spawn_barista_bot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', robot_name,
                   '-x', '0', '-y', '0', '-z', '0.2',
                   '-topic', 'robot_description'
        ]
    )


    return LaunchDescription(
        [gazebo_node, robot_state_publisher_node, joint_state_pulbisher_node, rviz_node, spawn_barista_bot]
    )

