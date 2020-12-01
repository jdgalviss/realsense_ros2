sleep 1
export DOMAIN_ID=0
export ROS_DOMAIN_ID=0
source /opt/ros/dashing/setup.bash
source /home/jdbot/repos/jetbot-ros2/dev_ws/install/setup.bash
# source /home/jdbot/repos/jetbot-ros2/scripts/env_vars.sh

ros2 run tf2_ros static_transform_publisher 0.0 0.025 0.03 -1.5708 0.0 -1.5708 camera_link_t265 camera_link_d435 & ros2 launch realsense_ros2 realsense_launch.py
