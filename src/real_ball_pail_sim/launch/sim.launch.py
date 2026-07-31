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
    pail_model_path = os.path.join(pkg_ball_pail, 'models', 'pail', 'model.sdf')
    ball_model_path = os.path.join(pkg_ball_pail, 'models', 'ball', 'model.sdf')

    with open(world_template, 'r') as f:
        world_sdf = f.read()

    # Dynamically inject launch arguments and model paths into the SDF
    world_sdf = world_sdf.replace('{h1}', str(h1)).replace('{h2}', str(h2)).replace('{s}', str(s))
    world_sdf = world_sdf.replace('{pail_model_path}', pail_model_path)
    world_sdf = world_sdf.replace('{ball_model_path}', ball_model_path)

    generated_world_path = os.path.join('/tmp', 'ball_and_pail_generated.sdf')
    with open(generated_world_path, 'w') as f:
        f.write(world_sdf)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': generated_world_path}.items()
    )

    # Node that publishes angle and velocity to ball
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

    # bridge ros and gazebo topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_ball_pail, 'config', 'ros_gz_bridge.yaml')
        }],
        output='screen'
    )

    return [gz_sim, bridge, ball_properties_publisher]

# generate ros2 launch description.
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('h1', default_value='2.0'),
        DeclareLaunchArgument('h2', default_value='1.0'),
        DeclareLaunchArgument('s', default_value='2.0'),
        DeclareLaunchArgument('v0', default_value='3.0'),
        DeclareLaunchArgument('g', default_value='9.81'),
        OpaqueFunction(function=launch_setup)
    ])
