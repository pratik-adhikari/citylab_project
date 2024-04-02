from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    # Get the path to the 'robot_patrol' package
    robot_patrol_pkg_dir = get_package_share_directory('robot_patrol')
    
    # Define the path to the RViz configuration file
    rviz_config_path = os.path.join(robot_patrol_pkg_dir, 'launch', 'scan1.rviz')

    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='direction_service',  
            name='direction_service_node',
            output='screen'
        ),
        Node(
            package='robot_patrol',
            executable='patrol_with_service',  
            name='patrol_with_service_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])
