import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    path_planner_dir = get_package_share_directory('path_planner_server')
    
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically start up the nav2 stack')
    
    # Get the behavior tree XML file
    behavior_tree_xml_path = os.path.join(
        path_planner_dir,
        'behavior_trees',
        'navigate_w_replanning_and_recovery.xml')
    
    # Determine which parameter files to use based on sim time
    bt_navigator_params = os.path.join(path_planner_dir, 'config', 'bt_navigator_sim.yaml')
    planner_params = os.path.join(path_planner_dir, 'config', 'planner_sim.yaml')
    controller_params = os.path.join(path_planner_dir, 'config', 'controller_sim.yaml')
    recoveries_params = os.path.join(path_planner_dir, 'config', 'recoveries_sim.yaml')
    
    # Select the correct command velocity topic for simulation or real robot
    cmd_vel_topic = 'cmd_vel'  # Default topic
    
    # Create a conditioned LaunchConfiguration for selecting real robot configs
    use_real_configs = PythonExpression(['"', use_sim_time, '" == "False"'])
    
    # Log the parameter file selection
    log_sim_params = LogInfo(msg="Using simulation parameter files")
    log_real_params = LogInfo(condition=IfCondition(use_real_configs), msg="Using real robot parameter files")
    
    # Switch parameter files and cmd_vel topic based on sim_time
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': behavior_tree_xml_path
    }
    
    # Rewritten YAMLs for parameter files
    bt_navigator_configured_params = RewrittenYaml(
        source_file=bt_navigator_params,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    planner_configured_params = RewrittenYaml(
        source_file=planner_params,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    controller_configured_params = RewrittenYaml(
        source_file=controller_params,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    recoveries_configured_params = RewrittenYaml(
        source_file=recoveries_params,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    # Get the RViz configuration file path
    rviz_config_file = os.path.join(path_planner_dir, 'rviz', 'pathplanning.rviz')
    
    # Create nodes
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
    
    bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_configured_params],
        remappings=[
            ('cmd_vel', PythonExpression([
                '"', cmd_vel_topic, '" + ("/diffbot_base_controller/cmd_vel_unstamped" if ', use_sim_time, ' == "True" else "/robot/cmd_vel")'
            ]))
        ])
    
    planner_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_configured_params],
        remappings=[
            ('cmd_vel', PythonExpression([
                '"', cmd_vel_topic, '" + ("/diffbot_base_controller/cmd_vel_unstamped" if ', use_sim_time, ' == "True" else "/robot/cmd_vel")'
            ]))
        ])
    
    controller_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_configured_params],
        remappings=[
            ('cmd_vel', PythonExpression([
                '"', cmd_vel_topic, '" + ("/diffbot_base_controller/cmd_vel_unstamped" if ', use_sim_time, ' == "True" else "/robot/cmd_vel")'
            ]))
        ])
    
    recoveries_cmd = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[recoveries_configured_params],
        remappings=[
            ('cmd_vel', PythonExpression([
                '"', cmd_vel_topic, '" + ("/diffbot_base_controller/cmd_vel_unstamped" if ', use_sim_time, ' == "True" else "/robot/cmd_vel")'
            ]))
        ])
    
    lifecycle_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_pathplanner',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['planner_server', 'controller_server', 'recoveries_server', 'bt_navigator']}
        ])
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    
    # Add logging for parameter file selection
    ld.add_action(log_sim_params)
    ld.add_action(log_real_params)
    
    # Add the nodes to the launch description
    ld.add_action(rviz_cmd)
    ld.add_action(bt_navigator_cmd)
    ld.add_action(planner_cmd)
    ld.add_action(controller_cmd)
    ld.add_action(recoveries_cmd)
    ld.add_action(lifecycle_cmd)
    
    return ld