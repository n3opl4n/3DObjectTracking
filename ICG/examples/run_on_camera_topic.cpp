// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include <filesystem/filesystem.h>
#include <icg/ros_camera.h>
#include <icg/basic_depth_renderer.h>
#include <icg/body.h>
#include <icg/common.h>
#include <icg/depth_modality.h>
#include <icg/depth_model.h>
#include <icg/normal_viewer.h>
#include <icg/region_modality.h>
#include <icg/region_model.h>
#include <icg/renderer_geometry.h>
#include <icg/static_detector.h>
#include <icg/tracker.h>

#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
int main(int argc, char *argv[]) {
  if (argc < 3) {
    std::cerr << "Not enough arguments: Provide directory and body_names";
    return 0;
  }
  const std::filesystem::path directory{argv[1]};
  std::vector<std::string> body_names;
  for (int i = 2; i < argc; ++i) {
    body_names.push_back(std::string{argv[i]});
  }

  constexpr bool kUseColorViewer = true;
  constexpr bool kUseDepthViewer = false;
  constexpr bool kMeasureOcclusions = true;
  constexpr bool kModelOcclusions = false;
  constexpr bool kVisualizePoseResult = false;
  constexpr bool kSaveImages = false;
  const std::filesystem::path save_directory{""};

  // Set up tracker and renderer geometry
  auto tracker_ptr{std::make_shared<icg::Tracker>("tracker")};
  auto renderer_geometry_ptr{
      std::make_shared<icg::RendererGeometry>("renderer geometry")};

  rclcpp::init(argc, argv);

  std::cout << "Creating node...\n";
//   rclcpp::Node::SharedPtr node = nullptr;
  auto node = rclcpp::Node::make_shared("run_on_camera_topic");
//   auto node2 = rclcpp::Node::make_shared("run_on_camera_topic2");

  std::cout << "Setting up cameras...\n";
  // Set up cameras
  auto color_camera_ptr{
      std::make_shared<icg::RosTopicColorCamera>(node, "azure_kinect_color")}; //ignore the "azure_kinect". It is just for internal use. Need to change it in config.
  
  auto depth_camera_ptr{
      std::make_shared<icg::RosTopicDepthCamera>(node, "azure_kinect_depth")};
  std::cout << "Camera subscribers ready.\n";

  while(!color_camera_ptr->gotInfo() || !depth_camera_ptr->gotInfo()) {
    rclcpp::spin_some(node);
    std::cout << "waiting for camera info messages...\n";
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  // Set up viewers
  if (kUseColorViewer) {
    auto color_viewer_ptr{std::make_shared<icg::NormalColorViewer>(
        "color_viewer", color_camera_ptr, renderer_geometry_ptr)};
    if (kSaveImages) color_viewer_ptr->StartSavingImages(save_directory, "bmp");
    tracker_ptr->AddViewer(color_viewer_ptr);
  }
  if (kUseDepthViewer) {
    auto depth_viewer_ptr{std::make_shared<icg::NormalDepthViewer>(
        "depth_viewer", depth_camera_ptr, renderer_geometry_ptr, 0.3f, 1.0f)};
    if (kSaveImages) depth_viewer_ptr->StartSavingImages(save_directory, "bmp");
    tracker_ptr->AddViewer(depth_viewer_ptr);
  }
  std::cout << "Viewers ready.\n";

  // Set up depth renderer
  auto color_depth_renderer_ptr{
      std::make_shared<icg::FocusedBasicDepthRenderer>(
          "color_depth_renderer", renderer_geometry_ptr, color_camera_ptr)};
  auto depth_depth_renderer_ptr{
      std::make_shared<icg::FocusedBasicDepthRenderer>(
          "depth_depth_renderer", renderer_geometry_ptr, depth_camera_ptr)};
  std::cout << "depth renderer ready.\n";


  for (const auto body_name : body_names) {
    // Set up body
    std::filesystem::path metafile_path{directory / (body_name + ".yaml")};
    auto body_ptr{std::make_shared<icg::Body>(body_name, metafile_path)};
    renderer_geometry_ptr->AddBody(body_ptr);
    color_depth_renderer_ptr->AddReferencedBody(body_ptr);
    depth_depth_renderer_ptr->AddReferencedBody(body_ptr);
    std::cout << "Body ready.\n";

    // Set up detector
    std::filesystem::path detector_path{directory /
                                        (body_name + "_detector.yaml")};
    auto detector_ptr{std::make_shared<icg::StaticDetector>(
        body_name + "_detector", detector_path, body_ptr)};
    tracker_ptr->AddDetector(detector_ptr);
    std::cout << "Detector ready.\n";

    // Set up models
    auto region_model_ptr{std::make_shared<icg::RegionModel>(
        body_name + "_region_model", body_ptr,
        directory / (body_name + "_region_model.bin"))};
    auto depth_model_ptr{std::make_shared<icg::DepthModel>(
        body_name + "_depth_model", body_ptr,
        directory / (body_name + "_depth_model.bin"))};
    std::cout << "Models ready.\n";

    // Set up modalities
    auto region_modality_ptr{std::make_shared<icg::RegionModality>(
        body_name + "_region_modality", body_ptr, color_camera_ptr,
        region_model_ptr)};
    auto depth_modality_ptr{std::make_shared<icg::DepthModality>(
        body_name + "_depth_modality", body_ptr, depth_camera_ptr,
        depth_model_ptr)};
    if (kVisualizePoseResult) {
      region_modality_ptr->set_visualize_pose_result(true);
    }
    if (kMeasureOcclusions) {
      region_modality_ptr->MeasureOcclusions(depth_camera_ptr);
      depth_modality_ptr->MeasureOcclusions();
    }
    if (kModelOcclusions) {
      region_modality_ptr->ModelOcclusions(color_depth_renderer_ptr);
      depth_modality_ptr->ModelOcclusions(depth_depth_renderer_ptr);
    }
    std::cout << "Modalities ready.\n";

    // Set up optimizer
    auto body1_optimizer_ptr{
        std::make_shared<icg::Optimizer>(body_name + "_optimizer")};
    body1_optimizer_ptr->AddModality(region_modality_ptr);
    body1_optimizer_ptr->AddModality(depth_modality_ptr);
    tracker_ptr->AddOptimizer(body1_optimizer_ptr);
    std::cout << "Optimizer ready.\n";
  }
  std::cout << "Settin up...\n";

  tracker_ptr->setRosNode(node);
  if (!tracker_ptr->SetUp()) 
  {
    std::cout << "Could not SetUp...\n";
    return 0;
  }
  
  std::cout << "Processing...\n";
  if (!tracker_ptr->RunTrackerProcessRos(true, false)) 
  {
    std::cout << "Could not RunTrackerProcess...\n";
    return 0;
  }
  return 0;
}
