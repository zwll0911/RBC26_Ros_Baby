import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    pkg_name = 'my_robot_controller' 

    # --- 1. DEFINE THE LAUNCH ARGUMENT ---
    # This creates a flag called 'slam'. Default is False (Navigation Mode).
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Set to True to run Mapping, False for Navigation'
    )
    
    # Read the value of the argument
    is_slam = LaunchConfiguration('slam')

    # --- 2. DEFINE FILE PATHS ---
    robot_launch_path = os.path.join(
        get_package_share_directory(pkg_name), 'launch', 'robot.launch.py')
    
    mapping_launch_path = os.path.join(
        get_package_share_directory(pkg_name), 'launch', 'mapping.launch.py')
    
    localization_launch_path = os.path.join(
        get_package_share_directory(pkg_name), 'launch', 'localization.launch.py')
    
    navigation_launch_path = os.path.join(
        get_package_share_directory(pkg_name), 'launch', 'navigation.launch.py')

    # --- 3. CREATE LAUNCH ACTIONS ---

    # A. Robot Hardware (ALWAYS runs)
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch_path)
    )

    # B. Mapping (Only runs IF slam=True)
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch_path),
        condition=IfCondition(is_slam) 
    )

    # C. Localization (Only runs UNLESS slam=True)
    localization_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_launch_path),
                condition=UnlessCondition(is_slam)
            )
        ]
    )

    # D. Navigation (Only runs UNLESS slam=True)
    navigation_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch_path),
                condition=UnlessCondition(is_slam)
            )
        ]
    )

    # --- 4. RETURN EVERYTHING ---
    return LaunchDescription([
        slam_arg,            # Register the argument
        robot_launch,        # Always start the robot
        mapping_launch,      # Maybe start mapping
        localization_launch, # Maybe start AMCL
        navigation_launch    # Maybe start Nav2
    ])