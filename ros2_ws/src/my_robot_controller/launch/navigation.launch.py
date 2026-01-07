import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_robot_controller'
    
    # Path to our YAML file
    params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'nav2_params.yaml'
    )

    # The list of nodes we want to manage (No collision_monitor!)
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower'
    ]

    return LaunchDescription([
        # 1. Controller Server (Follows the path)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': False}]
        ),

        # 2. Planner Server (Calculates the path)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': False}]
        ),

        # 3. Behavior Server (Recovers if stuck)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': False}]
        ),

        # 4. BT Navigator ( The "Brain" )
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': False}]
        ),
        
        # 5. Waypoint Follower (Optional, but good to have)
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file, {'use_sim_time': False}]
        ),

        # 6. Lifecycle Manager (Turns everything on)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'autostart': True},
                {'node_names': lifecycle_nodes}
            ]
        ),
    ])