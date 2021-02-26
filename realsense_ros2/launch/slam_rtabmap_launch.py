import launch
import launch.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    parameters=[{
          'queue_size':20,
          'frame_id':'camera_link_d435',
          'use_sim_time':use_sim_time,
          'subscribe_depth':True}]

    remappings=[
          ('odom', 'rs_t265/odom'),
          ('rgb/image', '/rs_d435/image_raw'),
          ('rgb/camera_info', 'rs_d435/image_raw/camera_info'),
          ('depth/image', '/rs_d435/aligned_depth/image_raw')]

    return LaunchDescription([
        #         Node(
        #     package='realsense_ros2',
        #     node_executable='rs_t265_node',
        #     node_name='rs_t265',
        #     output='screen'
        # ),
        # Node(
        #     package='realsense_ros2',
        #     node_executable='rs_d435_node',
        #     node_name='rs_d435',
        #     output='screen',
        #     parameters=[
        #         {"publish_depth": True},
        #         {"publish_pointcloud": False},
        #         {"is_color": False},
        #         {"publish_image_raw_": True},
        #         {"fps": 15}      # Can only take values of 6,15,30 or 60
        #     ]
        # ),
        # Node(
        #     ## Configure the TF of the robot to the origin of the map coordinates
        #     package='tf2_ros',
        #     node_executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0.0', '0.025', '0.03', '-1.5708', '0.0', '-1.5708', 'camera_link_t265', 'camera_link_d435']
        #     ),

        # Node(
        #     ## Configure the TF of the robot to the origin of the map coordinates
        #     package='tf2_ros',
        #     node_executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0.0', '0.025', '0.03', '0.0', '0.0', '0.0', 'camera_link_t265', 'camera_link_d435b']
        #     ),

        # Node(
        #     ## Configure the TF of the robot to the origin of the map coordinates
        #     package='tf2_ros',
        #     node_executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['-0.15', '0.0', '0.0', '0.0', '0.0', '0.0', 'camera_link_t265', 'base_link']
        # ),

        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Nodes to launch
        Node(
            package='rtabmap_ros', node_executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        # Node(
        #     package='rtabmap_ros', node_executable='rtabmapviz', output='screen',
        #     parameters=parameters,
        #     remappings=remappings),

    ])