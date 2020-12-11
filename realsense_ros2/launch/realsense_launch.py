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
                {"publish_depth": True},
                {"publish_pointcloud": True},
                {"is_color": False},
                {"publish_image_raw_": True},
                {"fps": 15}      # Can only take values of 6,15,30 or 60
            ]
        ),
        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.025', '0.03', '-1.5708', '0.0', '-1.5708', 'camera_link_t265', 'camera_link_d435']
            ),
        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['-0.15', '0.0', '0.0', '0.0', '0.0', '0.0', 'camera_link_t265', 'base_link']
        )
    ])
