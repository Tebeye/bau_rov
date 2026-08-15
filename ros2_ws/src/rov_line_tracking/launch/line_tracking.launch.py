import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rov_line_tracking')
    default_params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS 2 parameters YAML file to load'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    vision_node = Node(
        package='rov_line_tracking',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    control_node = Node(
        package='rov_line_tracking',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        vision_node,
        control_node
    ])
