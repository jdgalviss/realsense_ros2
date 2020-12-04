import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    realsense_prefix = get_package_share_directory('realsense_ros2')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
                                                    default=os.path.join(realsense_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='rs_cartographer.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),

        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

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
                {"is_color": False},
                {"publish_depth": True},
                {"publish_pointcloud": False},
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
            arguments=['0.0', '0.025', '0.03', '0.0', '0.0', '0.0', 'camera_link_t265', 'camera_link_d435_scan']
            ),

        Node(
            package='depthimage_to_laserscan',
            node_executable='depthimage_to_laserscan_node',
            node_name='scan',
            output='screen',
            parameters=[{'output_frame':'camera_link_t265'}],
            remappings=[('depth','rs_d435/aligned_depth/image_raw'),
                        ('depth_camera_info', 'rs_d435/aligned_depth/camera_info')],
            ),

        Node(
            package='cartographer_ros',
            node_executable='cartographer_node',
            output='log',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir, '-configuration_basename', configuration_basename],
            remappings=[('odom','rs_t265/odom')]),

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
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]),
    ])
