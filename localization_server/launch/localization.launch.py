import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the directory path for the package
    pkg_share = get_package_share_directory('localization_server')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'localization.rviz')
    
    # Declare launch arguments
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml',
        description='Name of the map file to load'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )

    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='amcl_config_sim.yaml',
        description='Localization config file based on map'
    )

    # Extract launch arguments
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')  
    localization_file = PythonExpression([
        "'amcl_config_sim.yaml' if 'warehouse_map_sim.yaml' in '", map_file, 
        "' else 'amcl_config_real.yaml'"
    ])    
    map_yaml_file = PathJoinSubstitution([
        FindPackageShare('map_server'),
        'config',
        map_file
    ])

    amcl_yaml_file = PathJoinSubstitution([
        FindPackageShare('localization_server'),
        'config',
        localization_file
    ])

    # Map Server Node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml_file}
        ]
    )
    
    # AMCL Node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_yaml_file]
    )
    
    # Lifecycle Manager Node
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )
    
    # RViz Node
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
        map_file_arg,
        localization_arg,
        use_sim_time_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        rviz_node
    ])