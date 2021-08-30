[//]: # (Image References)

[image1]: imgs/rs-viewer.png "rs-viewer"
[image2]: imgs/rviz.gif "rviz"
[image3]: imgs/cartographer.png "cartographer"
[image4]: imgs/cartographer.gif "2D"
[image5]: imgs/rtabmap.gif "rtabmap"
[image6]: imgs/rtabmap.png "rtabmap"





# realsense_ros2
Ros 2 wrapper for intel realsense cameras d435 and t265.

This wrapper's implementation is specially developed with the objective of running it in Nvidia's Jetson Nano, however it should also work on any other platform running Ubuntu 18.04 and 20.04.

By running this wrapper you would be able to obtain:

* Pose data from the realsense t265 tracking camera
* Pointcloud from the realsense d435 depth stereo camera
* Depth Image from the realsense d435 depth stereo camera

**Tested on Jetson Nano:
L4T 32.4.3 [ JetPack 4.4 ]
   Ubuntu 18.04.4 LTS
   Kernel Version: 4.9.140-tegra 
   ROS 2 Eloquent**

**Tested on Ubuntu 20.04 amd64:
ROS2 Foxy**

## Requirements
* ROS2 eloquent or foxy

## Installation
1. Install librealsense2 as per the official [instructions](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md).
2. Connect your cameras and check they are working propperly by openning a new terminal and typing:

    ```bash
    realsense-viewer
    ```
    You should be able to add the t265 and d435 cameras and view their data streams simultaneously.

    ![rs-viewer][image1]

3. Create a ROS 2 workspace, clone this repository inside and build: 
    ```bash
    mkdir -p dev_ws/src
    cd dev_ws/src
    git clone https://github.com/jdgalviss/realsense_ros2.git
    cd ..
    colcon build
    ```
4. Source your workspace
    ```bash
    . install/setup.bash
    ```

## Run
### D435 depth camera only
Run the node with the following command:

```bash
ros2 run realsense_ros2 rs_d435_node --ros-args -p is_color:=true -p publish_depth:=true -p fps:=30
```

* When the *is_color* parameter is set to true, the color image from the depth camera is aligned with the pointcloud.
* When the *publish_depth* parameter is set to true, the depth image is published.
* When the *publish_pointcloud* parameter is set to true, the 3D pointcloud is published.
* The *fps* parameter is used to modify the rate at which the node publishes the pointcloud and the depth image.


### T265 tracking camera only
Run the node with the following command:

```bash
ros2 run realsense_ros2 rs_t265_node
```
### T265 tracking and D435 depth cameras simultaneously
In one terminal, launch the two cameras:
```bash
ros2 launch realsense_ros2 realsense_launch.py
```
![rviz][image2]

### T265 tracking and D435 depth cameras simultaneously with SLAM-TOOLBOX 2D SLAM
In one terminal, launch the 2 cameras:
```bash
ros2 launch realsense_ros2 realsense_2d_slam_launch.py
```

In another terminal launch slam-toolbox:
```bash
ros2 launch realsense_ros2 online_async_launch.py
```

This requires to have ROS Slam-toolbox installed (apt-get install ros-<distro>-slam-toolbox)

![cartographer][image4]

### T265 tracking and D435 depth cameras simultaneously with rtabmap 3D SLAM
First, Download and Install rtabmap and rtabmap_ros following these [instructions](https://github.com/introlab/rtabmap_ros/tree/ros2#rtabmap_ros) in the branch **ros2**.

In one terminal, launch the two cameras and rtabmap ros (make sure that you source the workspace where rtabmap_ros was built):
```bash
ros2 launch realsense_ros2 slam_rtabmap_launch.py
```
![rtabmap][image5]
![rtabmap2][image6]


## Published topics

### rs_t265_node
* rs_t265/odom [nav_msgs/Odometry]: Pose and speeds of the t265 tracking camera.
* rs_t265/imu [sensor_msgs/Imu]: Imu data.
* tf


### rs_d435_node

* rs_d435/point_cloud [sensor_msgs/PointCloud2]: Pointcloud from d435 depth camera.
* rs_d435/aligned_depth/image_raw [sensor_msgs/Image]: Depth Image from d435 depth camera (only published if *publish_depth* parameter is set to true ).
* rs_d435/image_raw [sensor_msgs/Image]: Raw Color images

