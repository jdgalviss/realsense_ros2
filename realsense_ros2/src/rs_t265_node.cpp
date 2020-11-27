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
#include <tf2_msgs/msg/tf_message.hpp>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

/*! T265 Node class */
class T265Node : public rclcpp::Node
{
  public:
    T265Node()
    : Node("t265_node"), tf_broadcaster_(this)
    {
      // Define configuration to start stream from t265 camera
      cfg_.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
      // Start pipeline with chosen configuration
      pipe_.start(cfg_);

      // Publishers
      odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("rs_t265/odom", 10);

      // Timer used to publish camera's odometry periodically
      timer_ = this->create_wall_timer(
      100ms, std::bind(&T265Node::TimerCallback, this));
    }

  private:
    void TimerCallback()
    {
      // Wait for the next set of frames from the camera
      auto frames = pipe_.wait_for_frames();
      // Get a frame from the pose stream
      auto f = frames.first_or_default(RS2_STREAM_POSE);
      // Cast the frame to pose_frame and get its data
      auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

      // Create odometry msg and publish
      auto odom_msg = nav_msgs::msg::Odometry();
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = "camera_link_t265";
      odom_msg.header.stamp = rclcpp::Clock().now();
      odom_msg.pose.pose.position.x = pose_data.translation.x;
      odom_msg.pose.pose.position.y = pose_data.translation.y;
      odom_msg.twist.twist.linear.z = pose_data.translation.z;

      odom_msg.pose.pose.orientation.x = -pose_data.rotation.z;
      odom_msg.pose.pose.orientation.y = -pose_data.rotation.x;
      odom_msg.pose.pose.orientation.z = pose_data.rotation.y;
      odom_msg.pose.pose.orientation.w = pose_data.rotation.w;

      odom_msg.twist.twist.linear.x = pose_data.velocity.x;
      odom_msg.twist.twist.linear.y = pose_data.velocity.y;
      odom_msg.twist.twist.linear.z = pose_data.velocity.z;

      odom_msg.twist.twist.angular.x = pose_data.angular_velocity.x;
      odom_msg.twist.twist.angular.y = pose_data.angular_velocity.y;
      odom_msg.twist.twist.angular.z = pose_data.angular_velocity.z;
      odom_publisher_->publish(odom_msg);

      // Publish tf data
      tf2_msgs::msg::TFMessage tfs;
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = rclcpp::Clock().now();
      tf.header.frame_id = "odom";
      tf.child_frame_id = "camera_link_t265";
      tf.transform.translation.x = pose_data.translation.x;
      tf.transform.translation.y = pose_data.translation.y;
      tf.transform.translation.z = pose_data.translation.z;
      tf.transform.rotation.x = -pose_data.rotation.z;
      tf.transform.rotation.y = -pose_data.rotation.x;
      tf.transform.rotation.z = pose_data.rotation.y;
      tf.transform.rotation.w = pose_data.rotation.w;
      tf_broadcaster_.sendTransform(tf);
      // set tf.transform
      //tfs.transforms.push_back(tf);

      // Publish transform
      //tf_publisher_->publish(tfs);
    }
    // Class Members
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe_;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    // rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_; // ROS dashing
    tf2_ros::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  printf("hello world rs_t265 package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<T265Node>());
  rclcpp::shutdown();
  return 0;
}
