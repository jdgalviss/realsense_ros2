/*
* ROS wrapper for Realsense t265 camera
* By: Juan Galvis
* https://github.com/jdgalviss
*
* This code is free software: you can redistribute it and/or modify
* it under the terms of the MIT License.
*
* This code is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
*/
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include <tf2/convert.h>

using namespace std::chrono_literals;

/*! T265 Node class */
class T265Node : public rclcpp::Node
{
public:
  T265Node()
      : Node("t265_node"), tf_broadcaster_(this)
  {
    //begin_ = std::chrono::steady_clock::now();
    // Define configuration to start stream from t265 camera
    cfg_.enable_stream(RS2_STREAM_ACCEL);
    cfg_.enable_stream(RS2_STREAM_GYRO);
    cfg_.enable_stream(RS2_STREAM_POSE);
    // Start pipeline with chosen configuration
    pipe_.start(cfg_);

    // Publishers
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("rs_t265/odom", 10);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("rs_t265/imu", 10);
    // Timer used to publish camera's odometry periodically
    timer_ = this->create_wall_timer(
        10ms, std::bind(&T265Node::TimerCallback, this)); //30ms
  }

private:
  void TimerCallback()
  {
    //std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    //RCLCPP_INFO(logger_, "t265 timer period: %d",std::chrono::duration_cast<std::chrono::milliseconds>(end - begin_).count());
    //begin_=end;

    // Wait for the next set of frames from the camera
    auto frameset = pipe_.wait_for_frames();

    // Find and retrieve IMU and/or tracking data

    if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO))
    {
      rs2_vector gyro_sample = gyro_frame.get_motion_data();
      imu_msg_.angular_velocity.x = gyro_sample.x;
      imu_msg_.angular_velocity.y = gyro_sample.y;
      imu_msg_.angular_velocity.z = gyro_sample.z;
    }

    if (rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL))
    {
      rs2_vector accel_sample = accel_frame.get_motion_data();
      imu_msg_.header.stamp = rclcpp::Clock().now();
      imu_msg_.header.frame_id = "t265_frame";
      imu_msg_.linear_acceleration.x = accel_sample.x;
      imu_msg_.linear_acceleration.y = accel_sample.y;
      imu_msg_.linear_acceleration.z = accel_sample.z;
      imu_publisher_->publish(imu_msg_);
    }

    

    if (rs2::pose_frame pose_frame = frameset.first_or_default(RS2_STREAM_POSE))
    {
      rs2_pose pose_data = pose_frame.get_pose_data();
      // Create odometry msg and publish
      auto odom_msg = nav_msgs::msg::Odometry();
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = "t265_frame";
      odom_msg.header.stamp = rclcpp::Clock().now();
      odom_msg.pose.pose.position.x = -pose_data.translation.z;
      odom_msg.pose.pose.position.y = -pose_data.translation.x;
      odom_msg.pose.pose.position.z = pose_data.translation.y;

      odom_msg.pose.pose.orientation.x = -pose_data.rotation.z;
      odom_msg.pose.pose.orientation.y = -pose_data.rotation.x;
      odom_msg.pose.pose.orientation.z = pose_data.rotation.y;
      odom_msg.pose.pose.orientation.w = pose_data.rotation.w;

      double r = 0.0f, p = 0.0f, y = 0.0f;
      tf2::Quaternion q(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      m.getRPY(r, p, y);

      double vel_x = -pose_data.velocity.z;
      double vel_y = -pose_data.velocity.x;
      odom_msg.twist.twist.linear.x = vel_x*cos(y) + vel_y*sin(y);
      odom_msg.twist.twist.linear.y = -vel_x*sin(y) + vel_y*cos(y);
      odom_msg.twist.twist.linear.z = pose_data.velocity.y;

      odom_msg.twist.twist.angular.x = -pose_data.angular_velocity.z;
      odom_msg.twist.twist.angular.y = -pose_data.angular_velocity.x;
      odom_msg.twist.twist.angular.z = pose_data.angular_velocity.y;
      odom_publisher_->publish(odom_msg);

      // Publish tf data
      tf2_msgs::msg::TFMessage tfs;
      geometry_msgs::msg::TransformStamped tf;
      
      tf.header.frame_id = "odom";
      tf.child_frame_id = "t265_frame";
      tf.transform.translation.x = -pose_data.translation.z;
      tf.transform.translation.y = -pose_data.translation.x;
      tf.transform.translation.z = pose_data.translation.y;
      tf.transform.rotation.x = -pose_data.rotation.z;
      tf.transform.rotation.y = -pose_data.rotation.x;
      tf.transform.rotation.z = pose_data.rotation.y;
      tf.transform.rotation.w = pose_data.rotation.w;
      tf.header.stamp = rclcpp::Clock().now();
      tf_broadcaster_.sendTransform(tf);
    }
  }
  // Class Members
  // std::chrono::steady_clock::time_point begin_;
  rclcpp::Logger logger_ = rclcpp::get_logger("T265Node");
  geometry_msgs::msg::TransformStamped tf_static_;
  sensor_msgs::msg::Imu imu_msg_; 
  bool publish_transform_to_depth_;
  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe_;
  // Create a configuration for configuring the pipeline with a non default profile
  rs2::config cfg_;
  rclcpp::TimerBase::SharedPtr timer_;
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char **argv)
{
  printf("starting rs_t265 node\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<T265Node>());
  rclcpp::shutdown();
  return 0;
}
