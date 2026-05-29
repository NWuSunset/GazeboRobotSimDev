import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    h1 = float(LaunchConfiguration('h1').perform(context))
    h2 = float(LaunchConfiguration('h2').perform(context))
    s = float(LaunchConfiguration('s').perform(context))
    v0 = float(LaunchConfiguration('v0').perform(context))
    g = float(LaunchConfiguration('g').perform(context))

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ball_pail = get_package_share_directory('real_ball_pail_sim')

    world_template = os.path.join(pkg_ball_pail, 'worlds', 'ball_and_pail.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_template}.items()
    )

    spawn_pail = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'pail',
            '-file', os.path.join(pkg_ball_pail, 'models', 'pail', 'model.sdf'),
            '-x', str(s), '-y', '0.0', '-z', str(h1)
        ],
        output='screen'
    )

    spawn_ball = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ball',
            '-file', os.path.join(pkg_ball_pail, 'models', 'ball', 'model.sdf'),
            '-x', '0.0', '-y', '0.0', '-z', str(h2)
        ],
        output='screen'
    )

    ball_properties_publisher = Node (
        package='real_ball_pail_sim',
        executable='ball_publisher',
        output='both',
        parameters=[{
            'h1': h1,
            'h2': h2,
            's': s,
            'v0': v0,
            'g': g,
        }],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_ball_pail, 'config', 'ros_gz_bridge.yaml')
        }],
        output='screen'
    )

    return [gz_sim, bridge, spawn_pail, spawn_ball, ball_properties_publisher]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('h1', default_value='2.0'),
        DeclareLaunchArgument('h2', default_value='1.0'),
        DeclareLaunchArgument('s', default_value='2.0'),
        DeclareLaunchArgument('v0', default_value='3.0'),
        DeclareLaunchArgument('g', default_value='9.81'),
        OpaqueFunction(function=launch_setup)
    ])
