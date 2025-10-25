import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    robot_name = 'carta_rob'
    package_name = 'carta'

    # ================== Load URDF/Xacro ==================
    model_file = os.path.join(get_package_share_directory(package_name), 'model', 'carta.urdf.xacro')
    robot_description_config = xacro.process_file(model_file)
    robot_description = robot_description_config.toxml()

    # ================== Gazebo World ==================
    world_file = os.path.join(get_package_share_directory(package_name), 'worlds', 'new.world')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'gui': 'true'}.items()
    )

    # ================== Robot State Publisher ==================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )
    joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    output='screen',
    parameters=[{'use_sim_time': True}]
    )
    # ================== Spawn Entity ==================
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robot_name],
        output='screen'
    )

    # ================== RViz ==================
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'new.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_launch)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(rviz_node)
    ld.add_action(joint_state_publisher_node)
    return ld