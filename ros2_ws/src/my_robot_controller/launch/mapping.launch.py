import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Path to your config file (The one where we set throttle_scans: 2)
    mapper_params_file = os.path.join(
        get_package_share_directory('my_robot_controller'), 'config', 'mapper_params_online_async.yaml'
    )

    # Launch SLAM Toolbox with the config file
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={'slam_params_file': mapper_params_file}.items()
    )

    return LaunchDescription([
        slam_launch
    ])
