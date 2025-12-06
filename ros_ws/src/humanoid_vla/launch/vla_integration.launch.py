from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Include individual component launch files
    voice_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('voice_interface'),
            '/launch/voice_interface.launch.py'
        ])
    )

    llm_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('llm_planner'),
            '/launch/llm_planner.launch.py'
        ])
    )

    cv_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('cv_detection'),
            '/launch/cv_detection.launch.py'
        ])
    )

    manipulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('manipulation'),
            '/launch/manipulation.launch.py'
        ])
    )

    # Nav2 launch (from our earlier implementation)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_config'),
            '/launch/nav2.launch.py'
        ])
    )

    # Launch description
    ld = LaunchDescription()

    # Add component launches
    ld.add_action(voice_interface_launch)
    ld.add_action(llm_planner_launch)
    ld.add_action(cv_detection_launch)
    ld.add_action(manipulation_launch)
    ld.add_action(nav2_launch)

    return ld