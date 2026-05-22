import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_ball_world(h1, h2, s, v0, g):
    delta_h = h1 - h2
    theta_rad = math.atan2(delta_h, s)
    vx0 = v0 * math.cos(theta_rad)
    vz0 = v0 * math.sin(theta_rad)

    pkg_project_gazebo = get_package_share_directory('ros_gz_gazebo')
    world_template = os.path.join(pkg_project_gazebo, 'worlds', 'ball_and_pail.sdf')
    with open(world_template, 'r', encoding='utf-8') as stream:
        world_contents = stream.read()

    world_contents = world_contents.replace(
        '<initial_velocity>0 -1.0 1.0 0 0 0</initial_velocity>',
        f'<initial_velocity>{vx0:.6f} 0.000000 {vz0:.6f} 0 0 0</initial_velocity>'
    )

    generated_world = os.path.join(tempfile.gettempdir(), 'ball_and_pail_launch.sdf')
    with open(generated_world, 'w', encoding='utf-8') as stream:
        stream.write(world_contents)

    return generated_world, theta_rad, vx0, vz0


def launch_setup(context, *args, **kwargs):
    h1 = float(LaunchConfiguration('h1').perform(context))
    h2 = float(LaunchConfiguration('h2').perform(context))
    s = float(LaunchConfiguration('s').perform(context))
    v0 = float(LaunchConfiguration('v0').perform(context))
    g = float(LaunchConfiguration('g').perform(context))

    generated_world, theta_rad, vx0, vz0 = _build_ball_world(h1, h2, s, v0, g)

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': generated_world}.items(),
    )

    ball_value_publisher = Node(
        package='ros_gz_application',
        executable='ball_pail_sub',
        name='ball_pail_planner',
        output='screen',
        parameters=[{
            'h1': h1,
            'h2': h2,
            's': s,
            'v0': v0,
            'g': g,
            'input_topic': 'ball_pail/input',
            'angle_topic': 'ball_pail/launch_angle',
            'velocity_topic': 'ball_pail/launch_velocity',
        }],
    )

    return [
        gz_sim,
        ball_value_publisher,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('h1', default_value='1.0'),
        DeclareLaunchArgument('h2', default_value='2.0'),
        DeclareLaunchArgument('s', default_value='2.0'),
        DeclareLaunchArgument('v0', default_value='1.0'),
        DeclareLaunchArgument('g', default_value='9.81'),
        OpaqueFunction(function=launch_setup),
    ])