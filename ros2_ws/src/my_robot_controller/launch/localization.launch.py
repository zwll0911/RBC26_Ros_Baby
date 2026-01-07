import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    package_name = 'my_robot_controller'
    
    # PATHS
    map_file_path = os.path.join(os.path.expanduser('~'), 'test_map.yaml')
    
    amcl_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'amcl_params.yaml'
    )

    # 1. MAP SERVER (Loads the map)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path}]
    )

    # 2. AMCL (Localizes the robot)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config]
    )

    # 3. LIFECYCLE MANAGER (Required for Nav2 nodes to start)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    return LaunchDescription([
        map_server_node,
        amcl_node,
        lifecycle_manager_node
    ])