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
            executable='rs_t265_node',
            name='rs_t265'
        ),
        Node(
            package='realsense_ros2',
            executable='rs_d435_node',
            name='rs_d435',
            parameters=[
                {"is_color": True},
                {"publish_depth": True},
                {"fps": 6}      # Can only take values of 6,15,30 or 60
            ]
        )
    ])