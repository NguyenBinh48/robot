import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_name = 'robot'

    sim_mode = LaunchConfiguration('sim_mode')
    sim_mode_dec = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Run in simulation (true) or on real robot (false)'
    )

    tracker_params_sim = os.path.join(
        get_package_share_directory(pkg_name), 'config', 'aruco_tracker_params_sim.yaml'
    )

    tracker_params_robot = os.path.join(
        get_package_share_directory(pkg_name), 'config', 'aruco_tracker_params_robot.yaml'
    )

    params_path = PythonExpression([
        '"', tracker_params_sim, '" if "true" == "', sim_mode, '" else "', tracker_params_robot, '"'
    ])

    tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(pkg_name), 'launch', 'aruco_tracker.launch.py')
        ]),
        launch_arguments={
            'params_file': params_path,
            'image_topic': '/camera/image_raw',
            # 'cmd_vel_topic': '/cmd_vel_tracker',
            'cmd_vel_topic': '/cmd_vel',
            'use_sim_time': sim_mode
        }.items()
    )

    return LaunchDescription([
        sim_mode_dec,
        tracker_launch,
    ])