import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_name = 'warebot'
    
    # 1. Path to your main Xacro file
    # This file imports all necessary description files
    xacro_file = os.path.join(
        get_package_share_directory(package_name),
        'description',
        'urdf',
        'warebot.urdf.xacro'
    )

    # 2. Robot State Publisher
    # This node converts the Xacro/URDF into the TF tree required by RViz
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file]),
                value_type=str
            ),
            'use_sim_time': False
        }]
    )

    # 3. Joint State Publisher GUI
    # Allows you to wiggle the wheels/joints manually to test physics
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file]),
                value_type=str
            )
        }]
    )

    # 4. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
    ])