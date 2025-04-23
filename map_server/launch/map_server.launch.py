import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the directory path
    pkg_share = get_package_share_directory('map_server')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'map_display.rviz')
    default_map_file = os.path.join(pkg_share, 'config', 'warehouse_map_sim.yaml')
    
    # Declare the map_file parameter
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml',
        description='Name of the map file to load'
    )
    
    map_file = LaunchConfiguration('map_file')
    
    # Determine the full path to the map file
    map_file_path = PathJoinSubstitution([
        FindPackageShare('map_server'),
        'config',
        map_file
    ])
    
    # Define the map_server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': True}, 
            {'yaml_filename':map_file_path} 
        ]
    )
    
    # Define the lifecycle_manager node
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )
    
    # Define the RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    # Create and return launch description
    return LaunchDescription([
        map_file_arg,
        map_server_node,
        lifecycle_manager_node,
        rviz_node
    ])