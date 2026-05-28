import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description(context):
    # get the initial parameters
    h1 = float(LaunchConfiguration('h1').perform(context))
    h2 = float(LaunchConfiguration('h2').perform(context))
    s = float(LaunchConfiguration('s').perform(context))
    v0 = float(LaunchConfiguration('v0').perform(context))
    g = float(LaunchConfiguration('g').perform(context))

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ball_pail = get_package_share_directory('real_ball_pail_sim')

    # load the sdf file for ball from description package
    sdf_file = os.path.join(pkg_ball_pail, 'models', 'ball', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    #setup sim to launch gazebo sim world
    world_template = os.path.join(pkg_ball_pail, 'worlds', 'ball_and_pail.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_template}.items()
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
            'input_topic': '/ball_pail/input',
            'angle_topic': '/ball_pail/launch_angle',
            'velocity_topic': '/ball_pail/launch_velocity',
        }],
    )

    #bridge ROS topics and Gazebo messages
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_ball_pail, 'config', 'ros_gz_bridge.yaml')
        }],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('h1', default_value='2.0'),
        DeclareLaunchArgument('h2', default_value='1.0'),
        DeclareLaunchArgument('s', default_value='1.0'),
        DeclareLaunchArgument('v0', default_value='3.0'),
        DeclareLaunchArgument('g', default_value='9.81'),
        bridge,
        ball_properties_publisher
    ])
