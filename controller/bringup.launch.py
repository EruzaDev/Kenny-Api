"""Keep SLAM, pose, policy, landmark detection and missions alive together."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    python = LaunchConfiguration('python')
    environment, model = LaunchConfiguration('environment'), LaunchConfiguration('model')
    slam_launch = Path(get_package_share_directory('slam_toolbox')) / 'launch/online_async_launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('environment', description='Estimated or saved refined environment.json'),
        DeclareLaunchArgument('model', description='PPO checkpoint; loaded once'),
        DeclareLaunchArgument('python', default_value=str(root.parent / 'Kenny---Automotive-self-driving-trash-can/.venv/bin/python')),
        DeclareLaunchArgument('api', default_value='https://kenny-api.onrender.com'),
        DeclareLaunchArgument('motors', default_value='false', description='Run the USB encoder/motor bridge'),
        DeclareLaunchArgument('motor_port', default_value=''),
        DeclareLaunchArgument('markers', default_value='true'),
        DeclareLaunchArgument('initial_x', description='Physical starting x in the estimated map'),
        DeclareLaunchArgument('initial_y', description='Physical starting y in the estimated map'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(slam_launch)), launch_arguments={
            'slam_params_file': str(root / 'controller/slam.yaml'), 'use_sim_time': 'false',
            'autostart': 'true', 'use_lifecycle_manager': 'false'}.items()),
        ExecuteProcess(cmd=[python, '-m', 'controller.main', '--environment', environment, '--model', model,
            '--api', LaunchConfiguration('api'), '--initial-pose', LaunchConfiguration('initial_x'),
            LaunchConfiguration('initial_y'), LaunchConfiguration('initial_yaw')], cwd=str(root), output='screen'),
        ExecuteProcess(cmd=[python, '-m', 'controller.markers', '--environment', environment],
                       cwd=str(root), output='screen', condition=IfCondition(LaunchConfiguration('markers'))),
        ExecuteProcess(cmd=[python, '-m', 'controller.motors', '--port', LaunchConfiguration('motor_port')],
                       cwd=str(root), output='screen', condition=IfCondition(LaunchConfiguration('motors'))),
    ])
