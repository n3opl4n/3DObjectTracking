// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef ICG_INCLUDE_ICG_ROS_CAMERA_H_
#define ICG_INCLUDE_ICG_ROS_CAMERA_H_

#include <filesystem/filesystem.h>
#include <icg/camera.h>
#include <icg/common.h>

#include <chrono>
#include <iostream>
// #include <librealsense2/rs.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_utils_msgs/msg/simple_camera_info.hpp> //custom message for camera info
using std::placeholders::_1;

namespace icg {

/**
 * \brief Singleton class that allows getting data from a single RosTopic
 * instance and that is used by \ref RosTopicColorCamera and \ref
 * RosTopicDepthCamera.
 *
 * \details The method `UpdateCapture()` updates the `capture` object if
 * `UpdateCapture()` was already called with the same `id` before. If
 * `UpdateCapture()` was not yet called by the `id`, the same capture is used.
 * If the capture is updated, all memory values except for the `id` that called
 * the function are reset. All methods that are required to operate multiple
 * \ref Camera objects are thread-safe.
 */
class RosTopic {
 public:
  /// Singleton instance getter
  static RosTopic &GetInstance();
  RosTopic(const RosTopic &) = delete;
  void operator=(const RosTopic &) = delete;
  ~RosTopic();

  // Configuration and setup
  void UseColorCamera();
  void UseDepthCamera();
  int RegisterID();
  bool UnregisterID(int id);
  bool SetUp();

  // Main methods
  bool UpdateCapture(int id, bool synchronized);

  // Getters
  bool use_color_camera() const;
  bool use_depth_camera() const;
  // const rs2::frameset &frameset() const;
  // const rs2::pipeline_profile &profile() const;
  const Transform3fA *color2depth_pose() const;
  const Transform3fA *depth2color_pose() const;

  bool received_camera_info_;
  
  float depth_scale_temp_;
 private:
  RosTopic() = default;

  // Private data
  // rs2::config config_;
  // rs2::pipeline pipe_;
  std::map<int, bool> update_capture_ids_{};
  int next_id_ = 0;

  // Public data
  // rs2::pipeline_profile profile_;
  // rs2::frameset frameset_;
  Transform3fA color2depth_pose_{Transform3fA::Identity()};
  Transform3fA depth2color_pose_{Transform3fA::Identity()};

  // Internal state variables
  std::mutex mutex_;
  bool use_color_camera_ = false;
  bool use_depth_camera_ = false;
  bool initial_set_up_ = false;

};

/**
 * \brief \ref Camera that allows getting color images from a \ref RosTopic
 * camera.
 *
 * @param use_depth_as_world_frame specifies the depth camera frame as world
 * frame and automatically defines `camera2world_pose` as `color2depth_pose`.
 */
class RosTopicColorCamera : public ColorCamera {
 public:
  // Constructors, destructor, and setup method
  RosTopicColorCamera(const std::shared_ptr<rclcpp::Node> &node, 
                       const std::string &name,
                       bool use_depth_as_world_frame = false);
  RosTopicColorCamera(const std::shared_ptr<rclcpp::Node> &node, 
                       const std::string &name,
                       const std::filesystem::path &metafile_path);
  ~RosTopicColorCamera();

  void startSubscribers();

  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  // void CameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void CameraInfoCallback(const camera_utils_msgs::msg::SimpleCameraInfo::SharedPtr  msg);
  
  bool SetUp() override;

  // Setters
  void set_use_depth_as_world_frame(bool use_depth_as_world_frame);

  // Main method
  bool UpdateImage(bool synchronized) override;
  bool imageAvailable() {return got_frame_;};

  // Getters
  bool use_depth_as_world_frame() const;
  const Transform3fA *color2depth_pose() const;
  const Transform3fA *depth2color_pose() const;

  bool gotInfo() {return got_info_;};

 private:
  // Helper methods
  bool LoadMetaData();
  void GetIntrinsics();

  // Data
  RosTopic &realsense_;
  int realsense_id_{};
  bool use_depth_as_world_frame_ = false;
  bool initial_set_up_ = false;
  bool got_frame_ = false;
  bool got_info_;

  std::shared_ptr<rclcpp::Node> rclcpp_node_ptr_ = nullptr;
  // rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
  rclcpp::Subscription<camera_utils_msgs::msg::SimpleCameraInfo>::SharedPtr camera_info_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  cv_bridge::CvImagePtr topic_image_ptr_;
};

/**
 * \brief \ref Camera that allows getting depth images from a \ref RosTopic
 * camera.
 *
 * @param use_color_as_world_frame specifies the color camera frame as world
 * frame and automatically defines `camera2world_pose` as `depth2color_pose`.
 */
class RosTopicDepthCamera : public DepthCamera {
 public:
  // Constructors, destructor, and setup method
  RosTopicDepthCamera(const std::shared_ptr<rclcpp::Node> &node, 
                      const std::string &name,
                       bool use_color_as_world_frame = true);
  RosTopicDepthCamera(const std::shared_ptr<rclcpp::Node> &node, 
                      const std::string &name,
                       const std::filesystem::path &metafile_path);
  ~RosTopicDepthCamera();

  void startSubscribers();

  bool SetUp() override;

  void CameraInfoCallback(const camera_utils_msgs::msg::SimpleCameraInfo::SharedPtr msg);
  void DepthCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  // Setters
  void set_use_color_as_world_frame(bool use_color_as_world_frame);

  // Main method
  bool UpdateImage(bool synchronized) override;
  bool imageAvailable() {return got_frame_;};

  // Getters
  bool use_color_as_world_frame() const;
  const Transform3fA *color2depth_pose() const;
  const Transform3fA *depth2color_pose() const;

  bool gotInfo() {return got_info_;};

 private:
  // Helper methods
  bool LoadMetaData();
  void GetIntrinsics();

  // Data
  RosTopic &realsense_;
  int realsense_id_{};
  bool use_color_as_world_frame_ = true;
  bool initial_set_up_ = false;
  bool got_frame_ = false;
  bool got_info_;

  std::shared_ptr<rclcpp::Node> rclcpp_node_ptr_ = nullptr;
  rclcpp::Subscription<camera_utils_msgs::msg::SimpleCameraInfo>::SharedPtr camera_info_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber_;
  cv_bridge::CvImagePtr topic_depth_ptr_;
};

}  // namespace icg

#endif  // ICG_INCLUDE_ICG_ROS_CAMERA_H_
