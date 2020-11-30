import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense_ros2',
            node_executable='rs_t265_node',
            node_name='rs_t265',
            output='screen'
        ),
        Node(
            package='realsense_ros2',
            node_executable='rs_d435_node',
            node_name='rs_d435',
            output='screen',
            parameters=[
                {"is_color": True},
                {"publish_depth": False},
                {"fps": 6}      # Can only take values of 6,15,30 or 60
            ]
        )
    ])
