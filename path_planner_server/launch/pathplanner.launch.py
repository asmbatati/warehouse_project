import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('path_planner_server')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'pathplanning.rviz')
    
    # Declare the use_sim_time parameter
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    

    cmd_vel_topic = PythonExpression([
        "'/diffbot_base_controller/cmd_vel_unstamped' if '", use_sim_time, 
        "' == 'True' else '/cmd_vel'"
    ])

    controller_yaml_arg = DeclareLaunchArgument(
        'controller_yaml',
        default_value=PythonExpression([
            # note the outer quotes to make this a valid Python string:
            "'controller_sim.yaml' if '", use_sim_time,
            "' == 'True' else 'controller_real.yaml'"
        ]),
        description='controller_yaml'
    )

    bt_navigator_yaml_arg = DeclareLaunchArgument(
        'bt_navigator_yaml',
        default_value=PythonExpression([
            # note the outer quotes to make this a valid Python string:
            "'bt_navigator_sim.yaml' if '", use_sim_time,
            "' == 'True' else 'bt_navigator_real.yaml'"
        ]),
        description='bt_navigator_yaml'
    )

    planner_yaml_arg = DeclareLaunchArgument(
        'planner_yaml',
        default_value=PythonExpression([
            # note the outer quotes to make this a valid Python string:
            "'planner_sim.yaml' if '", use_sim_time,
            "' == 'True' else 'planner_real.yaml'"
        ]),
        description='planner_yaml'
    )

    recovery_yaml_arg = DeclareLaunchArgument(
        'recovery_yaml',
        default_value=PythonExpression([
            # note the outer quotes to make this a valid Python string:
            "'recoveries_sim.yaml' if '", use_sim_time,
            "' == 'True' else 'recoveries_real.yaml'"
        ]),
        description='recovery_yaml'
    )
    
    controller_yaml = LaunchConfiguration('controller_yaml')
    controller_yaml_file = PathJoinSubstitution([
        pkg_share,
        'config',
        controller_yaml
    ])
    bt_navigator_yaml = LaunchConfiguration('controller_yaml')
    bt_navigator_yaml_file = PathJoinSubstitution([
        pkg_share,
        'config',
        bt_navigator_yaml
    ])
    planner_yaml = LaunchConfiguration('planner_yaml')
    planner_yaml_file = PathJoinSubstitution([
        pkg_share,
        'config',
        planner_yaml
    ])
    recovery_yaml = LaunchConfiguration('recovery_yaml')
    recovery_yaml_file = PathJoinSubstitution([
        pkg_share,
        'config',
        recovery_yaml
    ])

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml_file],
        remappings=[('/cmd_vel', cmd_vel_topic)])
    
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml_file])

    recovery_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='recoveries_server',
        parameters=[recovery_yaml_file],
        output='screen')

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml_file])

    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_pathplanner',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['planner_server',
                                    'controller_server',
                                    'recoveries_server',
                                    'bt_navigator']}])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    return LaunchDescription([
        use_sim_time_arg,
        controller_yaml_arg,
        bt_navigator_yaml_arg,
        planner_yaml_arg,
        recovery_yaml_arg,
        controller_node,
        planner_node,
        recovery_node,
        bt_navigator_node,
        lifecycle_node,
        rviz_node
    ])