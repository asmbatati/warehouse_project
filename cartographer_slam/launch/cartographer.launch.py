import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    
    # Create LaunchConfiguration object
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Configuration directory and file
    configuration_directory = os.path.join(pkg_share, 'config')
    
    # Create a launch configuration for the configuration basename
    # The line below was problematic - we shouldn't do conditional logic this way
    # Instead, use launch substitutions properly
    configuration_basename_arg = DeclareLaunchArgument(
        'configuration_basename',
        default_value=PythonExpression([
            # note the outer quotes to make this a valid Python string:
            "'cartographer_sim.lua' if '", use_sim_time,
            "' == 'True' else 'cartographer_real.lua'"
        ]),
        description='Name of lua file for cartographer'
    )
    
    configuration_basename = LaunchConfiguration('configuration_basename')
    
    # Define the Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename
        ]
    )
    
    # Define the occupancy grid node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0'
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
        configuration_basename_arg,
        cartographer_node,
        occupancy_grid_node,
        rviz_node
    ])