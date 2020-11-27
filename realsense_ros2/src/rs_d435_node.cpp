/*
* ROS wrapper for Realsense d435 camera
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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/logger.hpp>
#include "realsense_ros2/constants.hpp"
#include <chrono>
#include <map>

#define IMAGE_FORMAT_DEPTH CV_16UC1 // CVBridge type
#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 480
#define DEPTH_FPS 6

using namespace std::chrono_literals;
using stream_index_pair = std::pair<rs2_stream, int>;

const stream_index_pair COLOR{RS2_STREAM_COLOR, 0};
const stream_index_pair DEPTH{RS2_STREAM_DEPTH, 0};
const stream_index_pair INFRA1{RS2_STREAM_INFRARED, 1};
const stream_index_pair INFRA2{RS2_STREAM_INFRARED, 2};
// const stream_index_pair FISHEYE{RS2_STREAM_FISHEYE, 0};
// const stream_index_pair GYRO{RS2_STREAM_GYRO, 0};
// const stream_index_pair ACCEL{RS2_STREAM_ACCEL, 0};

/*! dD35 Node class */
class D435Node : public rclcpp::Node
{
public:
  D435Node()
      : Node("d435_node")
  {
    // Check execution parameters
    this->declare_parameter<bool>("is_color", true);
    this->declare_parameter<bool>("publish_depth", true);

    this->get_parameter("is_color", is_color_);
    this->get_parameter("publish_depth", publish_depth_);

    if(is_color_)
      RCLCPP_INFO(logger_, "Holaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa=====");


    // Setup Device and Stream
    SetUpDevice();
    SetupStream();

    // Start pipeline with chosen configuration
    if (is_color_)
      pipe_.start();
    else
    {
      rs2::config cfg;
      cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 6);
      pipe_.start(cfg);
    }
    RCLCPP_INFO(logger_, "Capture Pipeline started!");

    // Publishers
    align_depth_publisher_ = image_transport::create_publisher(this, "aligned_depth_to_color/image_raw");
    align_depth_camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("aligned_depth_to_color/camera_info", 1);
    pcl_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);

    // Timer
    timer_ = this->create_wall_timer(200ms, std::bind(&D435Node::TimerCallback, this));
  }

private:
  void SetUpDevice()
  {
    try
    {
      ctx_.reset(new rs2::context());
      auto list = ctx_->query_devices();
      if (0 == list.size())
      {
        ctx_.reset();
        RCLCPP_ERROR(logger_, "No RealSense devices were found! Terminate RealSense Node...");
        rclcpp::shutdown();
        exit(1);
      }

      // Take the first device in the list.
      // Add an ability to get the specific device to work with from outside.
      dev_ = list.front();
      RCLCPP_INFO(logger_, "Device set up");
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(logger_, "An exception has been thrown: %s", ex.what());
      throw;
    }
    catch (...)
    {
      RCLCPP_ERROR(logger_, "Unknown exception has occured!");
      throw;
    }
  }

  void SetupStream()
  {
    // //Types for depth stream
    // rs2_format format = RS2_FORMAT_Z16;           // libRS type
    // int image_format = CV_16UC1;            // CVBridge type
    // std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1;         // ROS message type
    // int unit_step_size = sizeof(uint16_t);         // sensor_msgs::ImagePtr row step size
    // std::string stream_name = "depth";

    // Types for color stream
    rs2_format format = RS2_FORMAT_RGB8;                       // libRS type
    std::string encoding = sensor_msgs::image_encodings::RGB8; // ROS message type
    std::string stream_name = "color";

    std::string module_name = "0";

    auto dev_sensors = dev_.query_sensors();
    try
    {
      for (auto &&elem : dev_sensors)
      {
        module_name = elem.get_info(RS2_CAMERA_INFO_NAME);
        RCLCPP_INFO(logger_, "\n\nModule name: %s", module_name.c_str());
        auto sen = new rs2::sensor(elem);
        auto sens = std::unique_ptr<rs2::sensor>(sen);

        if ("Stereo Module" == module_name || "Coded-Light Depth Sensor" == module_name)
        {
          auto depth_sensor = sens->as<rs2::depth_sensor>();
          depth_scale_meters_ = depth_sensor.get_depth_scale();
        }
        auto profiles = sens->get_stream_profiles();
        for (auto &profile : profiles)
        {
          auto video_profile = profile.as<rs2::video_stream_profile>();

          RCLCPP_DEBUG(logger_, "Video profile found with  W: %d, H: %d, FPS: %d ", video_profile.width(),
                      video_profile.height(), video_profile.fps());

          // Choose right profile depending on parameters
          if (video_profile.format() == format &&
              video_profile.width() == DEPTH_WIDTH &&
              video_profile.height() == DEPTH_HEIGHT &&
              video_profile.fps() == DEPTH_FPS)
          {
            UpdateCalibData(video_profile);
            auto video_profile_ = video_profile;
            image_ =
                cv::Mat(video_profile.width(), video_profile.height(), format, cv::Scalar(0, 0, 0));
            RCLCPP_INFO(logger_, "%s stream is enabled - width: %d, height: %d, fps: %d",
                        module_name.c_str(), video_profile_.width(), video_profile_.height(), video_profile_.fps());
            break;
          }
        }
      }
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(logger_, "An exception has been thrown: %s", ex.what());
      throw;
    }
    catch (...)
    {
      RCLCPP_ERROR(logger_, "Unknown exception has occured!");
      throw;
    }
  }

  void UpdateCalibData(const rs2::video_stream_profile &video_profile)
  {
    stream_index_pair stream_index{video_profile.stream_type(), video_profile.stream_index()};
    auto intrinsic = video_profile.get_intrinsics();
    stream_intrinsics_ = intrinsic;
    camera_info_.width = intrinsic.width;
    camera_info_.height = intrinsic.height;
    camera_info_.header.frame_id = "camera_link_d435";

    camera_info_.k.at(0) = intrinsic.fx;
    camera_info_.k.at(2) = intrinsic.ppx;
    camera_info_.k.at(4) = intrinsic.fy;
    camera_info_.k.at(5) = intrinsic.ppy;
    camera_info_.k.at(8) = 1;

    camera_info_.p.at(0) = camera_info_.k.at(0);
    camera_info_.p.at(1) = 0;
    camera_info_.p.at(2) = camera_info_.k.at(2);
    camera_info_.p.at(3) = 0;
    camera_info_.p.at(4) = 0;
    camera_info_.p.at(5) = camera_info_.k.at(4);
    camera_info_.p.at(6) = camera_info_.k.at(5);
    camera_info_.p.at(7) = 0;
    camera_info_.p.at(8) = 0;
    camera_info_.p.at(9) = 0;
    camera_info_.p.at(10) = 1;
    camera_info_.p.at(11) = 0;

    if (stream_index == DEPTH)
    {
      rs2::stream_profile depth_profile;

      depth2color_extrinsics_ = depth_profile.get_extrinsics_to(video_profile);
      // set depth to color translation values in Projection matrix (P)
      camera_info_.p.at(3) = depth2color_extrinsics_.translation[0];  // Tx
      camera_info_.p.at(7) = depth2color_extrinsics_.translation[1];  // Ty
      camera_info_.p.at(11) = depth2color_extrinsics_.translation[2]; // Tz
    }
  }

  void PublishAlignedDepthImg()
  {
    // Get Depth frame
    // aligned_frameset_ = depth_frame.apply_filter(align);
    rs2::depth_frame aligned_depth = aligned_frameset_.get_depth_frame();

    // Transform to video frame
    auto vf = aligned_depth.as<rs2::video_frame>();
    auto depth_image = cv::Mat(cv::Size(vf.get_width(), vf.get_height()), IMAGE_FORMAT_DEPTH, const_cast<void *>(vf.get_data()), cv::Mat::AUTO_STEP);

    // Transform to ROS msg
    sensor_msgs::msg::Image::SharedPtr img;
    auto info_msg = camera_info_;
    img = cv_bridge::CvImage(
              std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depth_image)
              .toImageMsg();

    auto image = aligned_depth.as<rs2::video_frame>();
    auto bpp = image.get_bytes_per_pixel();
    auto height = image.get_height();
    auto width = image.get_width();
    img->width = width;
    img->height = height;
    img->is_bigendian = false;
    img->step = width * bpp;
    img->header.frame_id = "camera_link_d435";
    img->header.stamp = rclcpp::Clock().now();
    align_depth_publisher_.publish(img);
  }

  void publishAlignedPCTopic()
  {
    rs2::depth_frame aligned_depth = aligned_frameset_.get_depth_frame();
    auto image_depth16 = reinterpret_cast<const uint16_t *>(aligned_depth.get_data());
    auto depth_intrinsics = stream_intrinsics_;
    if (is_color_)
    {
      auto color_vf = aligned_frameset_.get_color_frame();
      image_ = cv::Mat(cv::Size(color_vf.get_width(), color_vf.get_height()), IMAGE_FORMAT_DEPTH,
                       const_cast<void *>(color_vf.get_data()), cv::Mat::AUTO_STEP);
    }
    unsigned char *color_data = image_.data;
    sensor_msgs::msg::PointCloud2 msg_pointcloud;
    msg_pointcloud.header.stamp = rclcpp::Clock().now();
    msg_pointcloud.header.frame_id = "camera_link_d435";
    msg_pointcloud.width = depth_intrinsics.width;
    msg_pointcloud.height = depth_intrinsics.height;
    msg_pointcloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);
    modifier.setPointCloud2Fields(3,
                                  "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                  "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    sensor_msgs::PointCloud2Iterator<float> iter_x(msg_pointcloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg_pointcloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg_pointcloud, "z");

    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg_pointcloud, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg_pointcloud, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg_pointcloud, "b");

    float std_nan = std::numeric_limits<float>::quiet_NaN();
    float depth_point[3], scaled_depth;
    // Fill the PointCloud2 fields
    for (int y = 0; y < depth_intrinsics.height; ++y)
    {
      for (int x = 0; x < depth_intrinsics.width; ++x)
      {
        scaled_depth = static_cast<float>(*image_depth16) * depth_scale_meters_;
        float depth_pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
        rs2_deproject_pixel_to_point(depth_point, &depth_intrinsics, depth_pixel, scaled_depth);
        auto iter_offset = x + y * depth_intrinsics.width;

        if (depth_point[2] <= 0.f || depth_point[2] > 5.f)
        {
          *(iter_x + iter_offset) = std_nan;
          *(iter_y + iter_offset) = std_nan;
          *(iter_z + iter_offset) = std_nan;
          *(iter_r + iter_offset) = static_cast<uint8_t>(96);
          *(iter_g + iter_offset) = static_cast<uint8_t>(157);
          *(iter_b + iter_offset) = static_cast<uint8_t>(198);
        }
        else
        {
          *(iter_x + iter_offset) = depth_point[0];
          *(iter_y + iter_offset) = depth_point[1];
          *(iter_z + iter_offset) = depth_point[2];
          if (is_color_)
          {
            *(iter_r + iter_offset) = color_data[iter_offset * 3];
            *(iter_g + iter_offset) = color_data[iter_offset * 3 + 1];
            *(iter_b + iter_offset) = color_data[iter_offset * 3 + 2];
          }
        }

        ++image_depth16;
      }
    }

    pcl_publisher_->publish(msg_pointcloud);
  }

  void TimerCallback()
  {
    auto frames = pipe_.wait_for_frames();
    // rs2::depth_frame depth_frame = frames.get_depth_frame();
    rs2::align align(RS2_STREAM_COLOR);
    aligned_frameset_ = frames.apply_filter(align);
    if (publish_depth_)
      PublishAlignedDepthImg();
    publishAlignedPCTopic();
  }
  rclcpp::Logger logger_ = rclcpp::get_logger("D435Node");
  rclcpp::TimerBase::SharedPtr timer_;
  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher_;
  image_transport::Publisher align_depth_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr align_depth_camera_info_publisher_;

  sensor_msgs::msg::CameraInfo camera_info_;

  rs2::device dev_;
  rs2_intrinsics depth_intrinsincs_;
  rs2_intrinsics stream_intrinsics_;
  rs2_extrinsics depth2color_extrinsics_;
  std::unique_ptr<rs2::context> ctx_;
  rs2::frameset aligned_frameset_;

  // rs2::video_stream_profile video_profile_;
  rs2::pipeline pipe_;
  cv::Mat image_;

  float depth_scale_meters_ = 1;
  bool is_color_ = true;
  bool publish_depth_ = true;
};

int main(int argc, char **argv)
{
  printf("hello world rs_d435 package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<D435Node>());
  rclcpp::shutdown();
  return 0;
}
