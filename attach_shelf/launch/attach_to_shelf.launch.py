import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    #files
    description_package_name = "attach_shelf"
    rviz_file = 'attach_shelf.rviz'

    obstacle = LaunchConfiguration("obstacle")
    obstacle_arg = DeclareLaunchArgument(
        "obstacle",
        default_value="0.5"
    )

    degrees = LaunchConfiguration("degrees")
    degrees_arg = DeclareLaunchArgument(
        "degrees",
        default_value="0.0"
    )

    final_approach = LaunchConfiguration("final_approach")
    final_approach_arg = DeclareLaunchArgument(
      "final_approach",
      default_value="False")

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
      "use_sim_time",
      default_value="True"
    )      
    
    attach_shelf_node = Node(
        package="attach_shelf",
        executable="pre_approach_v2",
        output="screen",
        parameters = [{
            "obstacle" : obstacle, 
            "degrees" : degrees, 
            "final_approach" : final_approach, 
            "use_sim_time" : use_sim_time
        }],
        emulate_tty=True
    )
    
    approach_service_server_node = Node(
        package="attach_shelf",
        executable="approach_service_server",
        output="screen",
                parameters = [{"use_sim_time" : use_sim_time}],
                emulate_tty=True
    )

    # Static transforms for missing frames
    static_transform_elevator = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0", "0", "0", "0", "0", "0", "1",  # Transform from parent to child
            "robot_evelator_base_link", "robot_evelator_platform_link"
        ],
        name="static_transform_elevator"
    )

    static_transform_left_wheel = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0", "0", "0", "0", "0", "0", "1",  # Transform from parent to child
            "robot_base_link", "robot_left_wheel_link"
        ],
        name="static_transform_left_wheel"
    )

    static_transform_right_wheel = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0", "0", "0", "0", "0", "0", "1",  # Transform from parent to child
            "robot_base_link", "robot_right_wheel_link"
        ],
        name="static_transform_right_wheel"
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(description_package_name), 'rviz', rviz_file)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        final_approach_arg,
        use_sim_time_arg,
        attach_shelf_node,
        approach_service_server_node,    
        static_transform_elevator,
        static_transform_left_wheel,
        static_transform_right_wheel,
        rviz_node,
    ])