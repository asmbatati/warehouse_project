import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the directory path
    pkg_share = get_package_share_directory('cartographer_slam')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'mapping.rviz')
    
    # Declare the use_sim_time parameter
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Determine which configuration file to use
    configuration_basename = 'cartographer_sim.lua'
    if use_sim_time.perform(None) == 'False':
        configuration_basename = 'cartographer_real.lua'
    
    configuration_directory = os.path.join(pkg_share, 'config')
    
    # Define the Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename,
            '--ros-args', '--log-level', 'INFO'
        ]
    )
    
    # Define the occupancy grid node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', '0.05',
            '--ros-args', '--log-level', 'INFO'
        ]
    )
    
    # Define the RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Create and return launch description
    return LaunchDescription([
        use_sim_time_arg,
        cartographer_node,
        occupancy_grid_node,
        rviz_node
    ])