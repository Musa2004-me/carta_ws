import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    robot_name="carta_rob"
    package_name='carta'  # <--- CHANGE ME

    world_file = os.path.join(get_package_share_directory(package_name), 'worlds', 'new.world')

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','carta_rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file, 'gui': 'true'}.items()
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robot_name],
        output='screen'
    )

    # Controllers
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    # RViz2 (with optional config)
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        "rviz",
        "view_config.rviz"
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file] if os.path.exists(rviz_config_file) else [],
    )
    # Cleanup: kill gzserver & gzclient on shutdown
    cleanup = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(cmd=["pkill", "-9", "gzserver"], shell=False),
                ExecuteProcess(cmd=["pkill", "-9", "gzclient"], shell=False),
            ]
        )
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        # diff_drive_spawner,
        # joint_broad_spawner,
        rviz,
        cleanup
    ])
