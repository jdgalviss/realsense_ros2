#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <algorithm>            // std::min, std::max

using namespace std::chrono_literals;


/*! dD35 Node class */
class D435Node : public rclcpp::Node
{
  public:
    D435Node()
    : Node("d435_node"), count_(0)
    {
      // Start streaming with default recommended configuration
      pipe_.start();
      
      // register callbacks to allow manipulation of the pointcloud
      // Publishers
      //odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("rs_d435/odom", 10);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&D435Node::TimerCallback, this));
    }

  private:
    void TimerCallback()
    {
      // Wait for the next set of frames from the camera
      try{
        auto frames = pipe_.wait_for_frames();

        auto color = frames.get_color_frame();

        // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
        if (!color)
          color = frames.get_infrared_frame();
        
        // Tell pointcloud object to map to this color frame
        pc_.map_to(color);

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points_ = pc_.calculate(depth);
        printf("hello world rs_d435 package\n");


      }
      catch (const std::exception & e)
      {
          std::cerr << e.what() << std::endl;
      }
    }
    // Class Members
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc_;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points_;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe_;

    size_t count_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  printf("hello world rs_d435 package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<D435Node>());
  rclcpp::shutdown();
  return 0;
}