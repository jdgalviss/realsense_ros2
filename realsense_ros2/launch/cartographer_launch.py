# Copyright (c) 2019 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# /* Author: Gary Liu */

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    realsense_prefix = get_package_share_directory('realsense_examples')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
                                                    default=os.path.join(realsense_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='rs_cartographer.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    rviz_config_dir = os.path.join(get_package_share_directory('realsense_ros'), 'config', 'rs_cartographer.rviz')

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
                {"publish_depth": True},
                {"publish_pointcloud": False},
                {"is_color": False},
                {"publish_image_raw_": False},
                {"fps": 30}      # Can only take values of 6,15,30 or 60
            ]
        ),

        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['-0.15', '0.0', '0.0', '0.0', '0.0', '0.0', 'camera_link_t265', 'base_link']
        ),
        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.025', '0.03', '0.0', '0.0', '0.0', 'camera_link_t265', 'camera_link_d435']
            ),


        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
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
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]),
    ])
