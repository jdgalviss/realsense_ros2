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
#include <image_transport/image_transport.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/logger.hpp>
#include "realsense_ros2/constants.hpp"
#include <chrono>
#include <map>
#include <tf2_ros/transform_listener.h>

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

/*! D435 Node class */
class D435Node : public rclcpp::Node
{
public:
  D435Node()
      : Node("d435_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    // Get execution parameters
    this->declare_parameter<bool>("is_color", false);
    this->declare_parameter<bool>("publish_depth", true);
    this->declare_parameter<bool>("publish_pointcloud", false);
    this->declare_parameter<bool>("publish_image_raw_", false);
    this->declare_parameter<int>("fps", 30); // can only take the values of
    this->get_parameter("is_color", is_color_);
    this->get_parameter("publish_depth", publish_depth_);
    this->get_parameter("publish_pointcloud", publish_pointcloud_);
    this->get_parameter("publish_image_raw_", publish_image_raw_);
    this->get_parameter("fps", fps_);

    begin_ = std::chrono::steady_clock::now();
    // Setup Device and Stream
    SetUpDevice();
    SetupStream();

    // Start pipeline with chosen configuration
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, fps_);
    if (is_color_ || publish_image_raw_)
      cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, fps_);
    pipe_.start(cfg);
    RCLCPP_INFO(logger_, "Capture Pipeline started!");

    // Publishers
    if (publish_depth_)
    {
      align_depth_publisher_ = image_transport::create_publisher(this, "rs_d435/aligned_depth/image_raw");
      depth_camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("rs_d435/aligned_depth/camera_info", 10);
    }
    if (publish_pointcloud_)
      pcl_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("rs_d435/point_cloud", 10);
    if (publish_image_raw_)
      image_raw_publisher_ = image_transport::create_publisher(this, "rs_d435/image_raw");


    // Timer
    timer_ = this->create_wall_timer(10ms, std::bind(&D435Node::TimerCallback, this));
  }

private:
  void SetUpDevice()
  {
    try
    {
      // Reset context
      ctx_.reset(new rs2::context());
      // query realsense devices
      auto list = ctx_->query_devices();
      if (0 == list.size())
      {
        ctx_.reset();
        RCLCPP_ERROR(logger_, "No RealSense devices were found! Terminate RealSense Node...");
        rclcpp::shutdown();
        exit(1);
      }
      // Front device corresponds to depth camera
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
    // Parameters of the video profile we want
    rs2_format format = RS2_FORMAT_RGB8;  
    rs2_format format_depth = RS2_FORMAT_Z16;                     // libRS type
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
        // From the Stereo module, get the depth scale parameter (extrinsic)
        if ("Stereo Module" == module_name || "Coded-Light Depth Sensor" == module_name)
        {
          auto depth_sensor = sens->as<rs2::depth_sensor>();
          depth_scale_meters_ = depth_sensor.get_depth_scale();
        }

        // Get the video profiles
        auto profiles = sens->get_stream_profiles();
        for (auto &profile : profiles)
        {
          auto video_profile = profile.as<rs2::video_stream_profile>();
          RCLCPP_DEBUG(logger_, "Video profile found with  format: %d, W: %d, H: %d, FPS: %d", video_profile.format(), video_profile.width(),
                       video_profile.height(), video_profile.fps());
          // Choose right profile depending on parameters
          if (video_profile.width() == DEPTH_WIDTH &&
              video_profile.height() == DEPTH_HEIGHT &&
              video_profile.fps() == fps_)
          {
            if(video_profile.format() == format){
              // Update calibration data with information from video profile
              UpdateCalibData(video_profile);
              video_profile_ = profile;
              image_ =
                  cv::Mat(video_profile.width(), video_profile.height(), format, cv::Scalar(0, 0, 0));
              RCLCPP_INFO(logger_, "%s stream is enabled - width: %d, height: %d, fps: %d",
                          module_name.c_str(), video_profile.width(), video_profile.height(), video_profile.fps());
              break;
            }
            else if (video_profile.format() == format_depth){
              depth_video_profile_ = profile;
            }
          }
        }

      }
      // UpdateCalibData(depth_video_profile_.as<rs2::video_stream_profile>());

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
    // Get profile intrinsics and save in camera_info msg
    auto intrinsic = video_profile.get_intrinsics();
    stream_intrinsics_ = intrinsic;
    if (stream_index == COLOR)
    {
      camera_info_depth_.width = intrinsic.width;
      camera_info_depth_.height = intrinsic.height;
      camera_info_depth_.header.frame_id = "camera_link_d435";
      camera_info_depth_.k.at(0) = intrinsic.fx;
      camera_info_depth_.k.at(2) = intrinsic.ppx;
      camera_info_depth_.k.at(4) = intrinsic.fy;
      camera_info_depth_.k.at(5) = intrinsic.ppy;
      camera_info_depth_.k.at(8) = 1;
      camera_info_depth_.p.at(0) = camera_info_depth_.k.at(0);
      camera_info_depth_.p.at(1) = 0;
      camera_info_depth_.p.at(2) = camera_info_depth_.k.at(2);
      camera_info_depth_.p.at(3) = 0;
      camera_info_depth_.p.at(4) = 0;
      camera_info_depth_.p.at(5) = camera_info_depth_.k.at(4);
      camera_info_depth_.p.at(6) = camera_info_depth_.k.at(5);
      camera_info_depth_.p.at(7) = 0;
      camera_info_depth_.p.at(8) = 0;
      camera_info_depth_.p.at(9) = 0;
      camera_info_depth_.p.at(10) = 1;
      camera_info_depth_.p.at(11) = 0;

      // depth2color_extrinsics_ = depth_video_profile_.get_extrinsics_to(video_profile_.as<rs2::video_stream_profile>());

      // set depth to color translation values in Projection matrix (P)
      // camera_info_depth_.p.at(3) = depth2color_extrinsics_.translation[0];  // Tx
      // camera_info_depth_.p.at(7) = depth2color_extrinsics_.translation[1];  // Ty
      // camera_info_depth_.p.at(11) = depth2color_extrinsics_.translation[2]; // Tz
      camera_info_depth_.distortion_model = "plumb_bob";

      // set R (rotation matrix) values to identity matrix
      camera_info_depth_.r.at(0) = 1.0;
      camera_info_depth_.r.at(1) = 0.0;
      camera_info_depth_.r.at(2) = 0.0;
      camera_info_depth_.r.at(3) = 0.0;
      camera_info_depth_.r.at(4) = 1.0;
      camera_info_depth_.r.at(5) = 0.0;
      camera_info_depth_.r.at(6) = 0.0;
      camera_info_depth_.r.at(7) = 0.0;
      camera_info_depth_.r.at(8) = 1.0;

      for (int i = 0; i < 5; i++)
      {
        camera_info_depth_.d.push_back(intrinsic.coeffs[i]);
      }
    }
  }

  void PublishAlignedDepthImg(const rclcpp::Time & t)
  {
    // Get Depth frame
    rs2::depth_frame aligned_depth = aligned_frameset_.get_depth_frame();

    // Transform to video frame
    auto vf = aligned_depth.as<rs2::video_frame>();
    auto depth_image = cv::Mat(cv::Size(vf.get_width(), vf.get_height()), IMAGE_FORMAT_DEPTH, const_cast<void *>(vf.get_data()), cv::Mat::AUTO_STEP);

    // Transform to ROS msg and publish
    sensor_msgs::msg::Image::SharedPtr img;
    img = cv_bridge::CvImage(
              std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depth_image)
              .toImageMsg();

    auto image = aligned_depth.as<rs2::video_frame>();
    auto bpp = image.get_bytes_per_pixel();
    auto height = image.get_height();
    auto width = image.get_width();
    img->header.stamp = t;
    img->width = width;
    img->height = height;
    img->is_bigendian = false;
    img->step = width * bpp;
    img->header.frame_id = "camera_link_d435";
    // Wait for transform to be available begfore publishing
    while (!tf_buffer_.canTransform("odom", "camera_link_t265", tf2::TimePointZero, 10s))
    {
    };

    align_depth_publisher_.publish(img);
    camera_info_depth_.header.stamp = t;
    depth_camera_info_publisher_->publish(camera_info_depth_);
  }

  void publishAlignedPCTopic(const rclcpp::Time & t)
  {
    // Get Depth Frame
    rs2::depth_frame aligned_depth = aligned_frameset_.get_depth_frame();
    auto image_depth16 = reinterpret_cast<const uint16_t *>(aligned_depth.get_data());
    auto depth_intrinsics = stream_intrinsics_;
    if (is_color_)
    {
      // If color is enabled, obtain color image to align it with pointcloud
      auto color_vf = aligned_frameset_.get_color_frame();
      image_ = cv::Mat(cv::Size(color_vf.get_width(), color_vf.get_height()), IMAGE_FORMAT_DEPTH,
                       const_cast<void *>(color_vf.get_data()), cv::Mat::AUTO_STEP);
    }
    if (publish_image_raw_)
    {
      auto image_frame = aligned_frameset_.get_color_frame();
        cv::Mat image_raw;
        image_raw = cv::Mat(cv::Size(image_frame.get_width(), image_frame.get_height()), CV_8UC3,
                            const_cast<void *>(image_frame.get_data()), cv::Mat::AUTO_STEP);
        sensor_msgs::msg::Image::SharedPtr img_msg;
        img_msg = cv_bridge::CvImage(
                      std_msgs::msg::Header(), sensor_msgs::image_encodings::RGB8, image_raw)
                      .toImageMsg();
        // auto image = aligned_depth.as<rs2::video_frame>();
        auto bpp = image_frame.get_bytes_per_pixel();
        auto width = image_frame.get_width();
        img_msg->width = width;
        img_msg->height = image_frame.get_height();
        img_msg->is_bigendian = false;
        img_msg->step = width * bpp;
        img_msg->header.frame_id = "camera_link_d435";
        img_msg->header.stamp = t;
        image_raw_publisher_.publish(img_msg);
    }
    unsigned char *color_data = image_.data;
    // Create pointcloud msg
    sensor_msgs::msg::PointCloud2 msg_pointcloud;
    msg_pointcloud.header.stamp = t;
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

        if (depth_point[2] <= 0.f || depth_point[2] > 3.5f)
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

  void publishRawImage(const rclcpp::Time & t)
  {
    if (rs2::video_frame image_frame = aligned_frameset_.first_or_default(RS2_STREAM_COLOR))
    {
      cv::Mat image_raw;
      image_raw = cv::Mat(cv::Size(image_frame.get_width(), image_frame.get_height()), CV_8UC3,
                          const_cast<void *>(image_frame.get_data()), cv::Mat::AUTO_STEP);
      sensor_msgs::msg::Image::SharedPtr img_msg;
      img_msg = cv_bridge::CvImage(
                    std_msgs::msg::Header(), sensor_msgs::image_encodings::RGB8, image_raw)
                    .toImageMsg();
      // auto image = aligned_depth.as<rs2::video_frame>();
      auto bpp = image_frame.get_bytes_per_pixel();
      auto width = image_frame.get_width();
      img_msg->width = width;
      img_msg->height = image_frame.get_height();
      img_msg->is_bigendian = false;
      img_msg->step = width * bpp;
      img_msg->header.frame_id = "camera_link_d435";
      img_msg->header.stamp = t;
      image_raw_publisher_.publish(img_msg);
    }
  }
  void TimerCallback()
  {
    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    // Wait for most recent frame
    auto frames = pipe_.wait_for_frames();
    auto time_stamp = rclcpp::Clock().now();
    if (is_color_ || publish_image_raw_){
      rs2::align align(RS2_STREAM_COLOR);
      aligned_frameset_ = frames.apply_filter(align);
    }
    else
    {
      aligned_frameset_ = frames;
    }
    
    //std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    //RCLCPP_INFO(logger_, "wait frames: %d",std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count());
    //begin=end;

    // If depth image is to be published, publish, otherwise only publish Pointcloud
    if (publish_pointcloud_)
      publishAlignedPCTopic(time_stamp);
    else
    {
      if (publish_image_raw_)
        publishRawImage(time_stamp);
    }
    if (publish_depth_)
      PublishAlignedDepthImg(time_stamp);
  }
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Logger logger_ = rclcpp::get_logger("D435Node");
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point begin_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher_;
  image_transport::Publisher align_depth_publisher_;
  image_transport::Publisher image_raw_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_publisher_;

  // Realsense variables
  rs2::device dev_;
  rs2_intrinsics depth_intrinsincs_;
  rs2_intrinsics stream_intrinsics_;
  rs2_extrinsics depth2color_extrinsics_;
  std::unique_ptr<rs2::context> ctx_;
  rs2::frameset aligned_frameset_;
  rs2::pipeline pipe_;
  rs2::stream_profile video_profile_;
  rs2::stream_profile depth_video_profile_;

  cv::Mat image_;
  float depth_scale_meters_ = 1;
  sensor_msgs::msg::CameraInfo camera_info_depth_;


  // Parameters
  bool is_color_ = true;
  bool publish_depth_ = true;
  bool publish_pointcloud_ = true;
  bool publish_image_raw_ = true;

  int fps_ = 30;
};

int main(int argc, char **argv)
{
  printf("hello world rs_d435 package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<D435Node>());
  rclcpp::shutdown();
  return 0;
}