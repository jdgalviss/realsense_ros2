import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

import os
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    realsense_prefix = get_package_share_directory('realsense_ros2')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
                                                    default=os.path.join(realsense_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='rs_cartographer.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
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
                {"publish_pointcloud": False},
                {"is_color": False},
                {"publish_image_raw_": True},
                {"fps": 6}      # Can only take values of 6,15,30 or 60
            ]
        ),

        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['-0.15', '0.0', '0.0', '0.0', '0.0', '0.0', 't265_frame', 'base_link']
        ),
        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.025', '0.03', '0.0', '0.0', '0.0', 'base_link', 'camera_link_d435']
            ),
            
        Node(
            package='depthimage_to_laserscan',
            node_executable='depthimage_to_laserscan_node',
            node_name='scan',
            output='screen',
            parameters=[{'output_frame':'camera_link_d435'}],
            remappings=[('depth','rs_d435/aligned_depth/image_raw'),
                        ('depth_camera_info', 'rs_d435/aligned_depth/camera_info')],
            ),

        Node(
            package='cartographer_ros',
            node_executable='cartographer_node',
            output='log',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir, '-configuration_basename', configuration_basename],
            remappings=[('odom','rs_t265/odom'),
            ('imu','rs_t265/imu')]),
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        Node(
            package='cartographer_ros',
            node_executable='occupancy_grid_node',
            node_name='occupancy_grid_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])
    ])