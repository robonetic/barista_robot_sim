import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
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

    return LaunchDescription(
        [robot_state_publisher_node, joint_state_pulbisher_node, rviz_node]
    )