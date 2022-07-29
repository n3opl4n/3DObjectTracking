// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include <icg/ros_camera.h>

namespace icg {

RosTopic &RosTopic::GetInstance() {
  static RosTopic realsense;
  return realsense;
}

RosTopic::~RosTopic() {
  // if (initial_set_up_) {
  //   pipe_.stop();
  // }
}

void RosTopic::UseColorCamera() { use_color_camera_ = true; }

void RosTopic::UseDepthCamera() { use_depth_camera_ = true; }

int RosTopic::RegisterID() {
  const std::lock_guard<std::mutex> lock{mutex_};
  update_capture_ids_.insert(std::pair<int, bool>{next_id_, true});
  return next_id_++;
}

bool RosTopic::UnregisterID(int id) {
  const std::lock_guard<std::mutex> lock{mutex_};
  return update_capture_ids_.erase(id);
}

bool RosTopic::SetUp() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!initial_set_up_) {
    auto qos = rclcpp::SensorDataQoS();
    image_subscriber_.subscribe(rclcpp_node_ptr_, "/ope_camera/color", qos.get_rmw_qos_profile());
    depth_subscriber_.subscribe(rclcpp_node_ptr_, "/ope_camera/depth", qos.get_rmw_qos_profile());
    sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(image_subscriber_, depth_subscriber_, 1);
    //sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(1), image_subscriber_, depth_subscriber_);
    sync_->registerCallback(&RosTopic::CameraCallback, this);
    got_frames_ = false;

    color2depth_pose_.setIdentity(); //already done by savvas in Publisher
    // Manual extrinsic calibration
    // Eigen::Matrix3f rot;
    // rot << 0.999836 , -0.0170622, -0.00614453,
    //        0.017079  ,  0.999851 , 0.00269125,
    //        0.00609769, -0.00279575 ,   0.999978;
     // Eigen::Vector3f trans;
    // trans << -0.01466 ,-2.06612e-05 ,-0.000497888;
    // color2depth_pose_.translate(trans);
    // color2depth_pose_.rotate(rot);
    depth2color_pose_ = color2depth_pose_.inverse();

    // Load multiple images to adjust to white balance
    // constexpr int kNumberImagesDropped = 10;
    // for (int i = 0; i < kNumberImagesDropped; ++i) {
    //   pipe_.try_wait_for_frames(&frameset_);
    // }
    initial_set_up_ = true;
    tic = std::chrono::high_resolution_clock::now();
  }
  return true;
}

void RosTopic::CameraCallback(const sensor_msgs::msg::Image::SharedPtr color_msg, const sensor_msgs::msg::Image::SharedPtr depth_msg) {
  auto elapsed_time{std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - tic)};
  std::cout << (int)(1000/elapsed_time.count()) << "\n";

  try
  {
  // topic_image_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  // topic_image_ptr_ = cv_bridge::toCvCopy(msg);
    topic_image_ptr_ = cv_bridge::toCvShare(color_msg);//, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    // ROS_ERROR("cv_bridge exception: %s", e.what());
    std::cout << "cv_bridge exception " << e.what() ;
    return;
  }

  try
  {
  // topic_depth_ptr_ = cv_bridge::toCvCopy(msg);//, sensor_msgs::image_encodings::MONO16);
    topic_depth_ptr_ = cv_bridge::toCvShare(depth_msg);//, sensor_msgs::image_encodings::);
  }
  catch (cv_bridge::Exception& e)
  {
    // ROS_ERROR("cv_bridge exception: %s", e.what());
    std::cout << "cv_bridge exception " << e.what() ;
    return;
  } 
  tic = std::chrono::high_resolution_clock::now();
  got_frames_ = true;
}

bool RosTopic::UpdateCapture(int id, bool synchronized) {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!initial_set_up_) return false;

  // if (update_capture_ids_.at(id)) {
  //   if (synchronized)
  //     pipe_.try_wait_for_frames(&frameset_);
  //   else
  //     pipe_.poll_for_frames(&frameset_);
  //   for (auto &[_, v] : update_capture_ids_) v = false;
  // }
  // update_capture_ids_.at(id) = true;
  return true;
}

bool RosTopic::use_color_camera() const { return use_color_camera_; }

bool RosTopic::use_depth_camera() const { return use_depth_camera_; }

// const rs2::frameset &RosTopic::frameset() const { return frameset_; }

// const rs2::pipeline_profile &RosTopic::profile() const { return profile_; }

const Transform3fA *RosTopic::color2depth_pose() const {
  if (initial_set_up_)
    return &color2depth_pose_;
  else
    return nullptr;
}

const Transform3fA *RosTopic::depth2color_pose() const {
  if (initial_set_up_)
    return &depth2color_pose_;
  else
    return nullptr;
}

RosTopicColorCamera::RosTopicColorCamera(const std::shared_ptr<rclcpp::Node> &node, 
                                           const std::string &name,
                                           bool use_depth_as_world_frame)
    : ColorCamera{name},
      use_depth_as_world_frame_{use_depth_as_world_frame},
      realsense_{RosTopic::GetInstance()} {
  realsense_.UseColorCamera();
  realsense_.PassRosNode(node);

  // realsense_id_ = realsense_.RegisterID();

  rclcpp_node_ptr_ = node;

  RosTopicColorCamera::startSubscribers();
}

RosTopicColorCamera::RosTopicColorCamera(const std::shared_ptr<rclcpp::Node> &node, 
                                           const std::string &name, 
                                           const std::filesystem::path &metafile_path)
    : ColorCamera{name, metafile_path}, 
      realsense_{RosTopic::GetInstance()} {
  realsense_.UseColorCamera();
  rclcpp_node_ptr_ = node;

  RosTopicColorCamera::startSubscribers();
}

RosTopicColorCamera::~RosTopicColorCamera() {
  // realsense_.UnregisterID(realsense_id_);
}

void RosTopicColorCamera::startSubscribers()
{
  got_info_ = false;
  
  // Create image subscriber
  camera_info_subscriber_ = rclcpp_node_ptr_->create_subscription<camera_utils_msgs::msg::SimpleCameraInfo>("/ope_camera/info", rclcpp::SensorDataQoS(),  std::bind(&RosTopicColorCamera::CameraInfoCallback, this, _1));
  // image_subscriber_ = rclcpp_node_ptr_->create_subscription<sensor_msgs::msg::Image>("/ope_camera/color", rclcpp::SensorDataQoS(),  std::bind(&RosTopicColorCamera::ImageCallback, this, _1));
}

void RosTopicColorCamera::ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) 
{
  if (got_info_)
  {
    try
    {
      // topic_image_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      // topic_image_ptr_ = cv_bridge::toCvCopy(msg);
      topic_image_ptr_ = cv_bridge::toCvShare(msg);//, sensor_msgs::image_encodings::BGR8);
     }
     catch (cv_bridge::Exception& e)
      {
        // ROS_ERROR("cv_bridge exception: %s", e.what());
        std::cout << "cv_bridge exception " << e.what() ;
        return;
      }

    got_frame_ = true;
  }
}


void RosTopicColorCamera::CameraInfoCallback(const camera_utils_msgs::msg::SimpleCameraInfo::SharedPtr msg) 
{
  if (!got_info_)
  {
    intrinsics_.fu = msg->fx; //intrinsics.fx;
    intrinsics_.fv = msg->fy; //intrinsics.fy;
    intrinsics_.ppu = msg->cx; //intrinsics.ppx;
    intrinsics_.ppv = msg->cy; //intrinsics.ppy;
    intrinsics_.width = msg->color_width;
    intrinsics_.height = msg->color_height;

    // realsense_.received_camera_info_ = true;
    got_info_ = true;
    // std::cout << "[RosTopicColorCamera::CameraInfoCallback] Saved camera info\n";
  }
}

bool RosTopicColorCamera::SetUp() {
  std::cout << "Setting up RosTopicColorCamera..."; 
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;
  if (!initial_set_up_ && !realsense_.SetUp()) return false;
  if (use_depth_as_world_frame_)
    set_camera2world_pose(*realsense_.color2depth_pose());
  // GetIntrinsics();
  SaveMetaDataIfDesired();
  set_up_ = true;
  initial_set_up_ = true;
  std::cout << "done\n";
  return true; //UpdateImage(true);
}

void RosTopicColorCamera::set_use_depth_as_world_frame(
    bool use_depth_as_world_frame) {
  use_depth_as_world_frame_ = use_depth_as_world_frame;
  set_up_ = false;
}

bool RosTopicColorCamera::UpdateImage(bool synchronized) {
  // std::cout << "Getting Color\n";
  if (!set_up_) {
    std::cerr << "Set up real sense color camera " << name_ << " first"
              << std::endl;
    return false;
  }

  if (!realsense_.got_frames_) {
    // std::cerr << "No color frame received yet" << std::endl;
    return false;
  }

  // image_ = realsense_.topic_image_ptr_->image; 
  cv::Mat{realsense_.topic_image_ptr_->image}.copyTo(image_);
  // Get frameset and copy data to image
  // realsense_.UpdateCapture(realsense_id_, synchronized);
  // cv::Mat{cv::Size{intrinsics_.width, intrinsics_.height}, CV_8UC3,
  //         (void *)realsense_.frameset().get_color_frame().get_data(),
  //         cv::Mat::AUTO_STEP}
  //     .copyTo(image_);

  SaveImageIfDesired();
  return true;
}

bool RosTopicColorCamera::use_depth_as_world_frame() const {
  return use_depth_as_world_frame_;
}

const Transform3fA *RosTopicColorCamera::color2depth_pose() const {
  return realsense_.color2depth_pose();
}

const Transform3fA *RosTopicColorCamera::depth2color_pose() const {
  return realsense_.depth2color_pose();
}

bool RosTopicColorCamera::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  ReadOptionalValueFromYaml(fs, "camera2world_pose", &camera2world_pose_);
  ReadOptionalValueFromYaml(fs, "save_directory", &save_directory_);
  ReadOptionalValueFromYaml(fs, "save_index", &save_index_);
  ReadOptionalValueFromYaml(fs, "save_image_type", &save_image_type_);
  ReadOptionalValueFromYaml(fs, "save_images", &save_images_);
  ReadOptionalValueFromYaml(fs, "use_depth_as_world_frame",
                            &use_depth_as_world_frame_);
  fs.release();

  // Process parameters
  if (save_directory_.is_relative())
    save_directory_ = metafile_path_.parent_path() / save_directory_;
  world2camera_pose_ = camera2world_pose_.inverse();
  return true;
}

void RosTopicColorCamera::GetIntrinsics() {
  // const rs2_intrinsics intrinsics = realsense_.profile()
  //                                       .get_stream(RS2_STREAM_COLOR)
  //                                       .as<rs2::video_stream_profile>()
  //                                       .get_intrinsics();
  // intrinsics_.fu = intrinsics.fx;
  // intrinsics_.fv = intrinsics.fy;
  // intrinsics_.ppu = intrinsics.ppx;
  // intrinsics_.ppv = intrinsics.ppy;
  // intrinsics_.width = intrinsics.width;
  // intrinsics_.height = intrinsics.height;
}

RosTopicDepthCamera::RosTopicDepthCamera(const std::shared_ptr<rclcpp::Node> &node, 
                                           const std::string &name,
                                           bool use_color_as_world_frame)
    : DepthCamera{name},
      use_color_as_world_frame_{use_color_as_world_frame},
      realsense_{RosTopic::GetInstance()} {
  realsense_.UseDepthCamera();

  rclcpp_node_ptr_ = node;
  RosTopicDepthCamera::startSubscribers();

}

RosTopicDepthCamera::RosTopicDepthCamera(const std::shared_ptr<rclcpp::Node> &node, 
                                         const std::string &name, 
                                         const std::filesystem::path &metafile_path)
    : DepthCamera{name, metafile_path}, realsense_{RosTopic::GetInstance()} {
  realsense_.UseDepthCamera();
  RosTopicDepthCamera::startSubscribers();
}

RosTopicDepthCamera::~RosTopicDepthCamera() {
}

void RosTopicDepthCamera::startSubscribers()
{
  got_info_ = false;

  // Info subscriber for depth image. In our case, depth is aligned with color so intrinsics are the same and the extrinsic matrix is identity
  camera_info_subscriber_ = rclcpp_node_ptr_->create_subscription<camera_utils_msgs::msg::SimpleCameraInfo>("/ope_camera/info", rclcpp::SensorDataQoS(),  std::bind(&RosTopicDepthCamera::CameraInfoCallback, this, _1));
  // depth_subscriber_ = rclcpp_node_ptr_->create_subscription<sensor_msgs::msg::Image>("/ope_camera/depth", rclcpp::SensorDataQoS(),  std::bind(&RosTopicDepthCamera::DepthCallback, this, _1));
}  

void RosTopicDepthCamera::CameraInfoCallback(const camera_utils_msgs::msg::SimpleCameraInfo::SharedPtr msg) 
{
  if (!got_info_)
  {
    intrinsics_.fu = msg->fx; //intrinsics.fx;
    intrinsics_.fv = msg->fy; //intrinsics.fy;
    intrinsics_.ppu = msg->cx; //intrinsics.ppx;
    intrinsics_.ppv = msg->cy; //intrinsics.ppy;
    intrinsics_.width = msg->color_width; //This is actually the depth (ignore the name)
    intrinsics_.height = msg->color_height; //This is actually the depth (ignore the name)
    depth_scale_ = 1/msg->depth_scale;

    got_info_ = true;
    // std::cout << "[RosTopicDepthCamera::CameraInfoCallback] Saved camera info\n";
    
  }
}

void RosTopicDepthCamera::DepthCallback(const sensor_msgs::msg::Image::SharedPtr msg) 
{
  if (got_info_)
  {
    // std::cout << "[RosTopicDepthCamera::DepthCallback] Getting depth\n";
    // depth_scale_ = realsense_.depth_scale_temp_; //save depth scale that we got from the /info topic
    
    try
    {
      // topic_depth_ptr_ = cv_bridge::toCvCopy(msg);//, sensor_msgs::image_encodings::MONO16);
      topic_depth_ptr_ = cv_bridge::toCvShare(msg);//, sensor_msgs::image_encodings::);
     }
     catch (cv_bridge::Exception& e)
      {
        // ROS_ERROR("cv_bridge exception: %s", e.what());
        std::cout << "cv_bridge exception " << e.what() ;
        return;
      }

    got_frame_ = true;
  }
}


bool RosTopicDepthCamera::SetUp() {
  std::cout << "Setting up RosTopicDepthCamera..."; 
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;
  // if (!initial_set_up_ && !realsense_.SetUp()) return false;
  if (use_color_as_world_frame_)
    set_camera2world_pose(*realsense_.depth2color_pose());
  // GetIntrinsics();
  SaveMetaDataIfDesired();
  set_up_ = true;
  initial_set_up_ = true;
  std::cout << "done\n";
  return true;//UpdateImage(true);
}

void RosTopicDepthCamera::set_use_color_as_world_frame(
    bool use_color_as_world_frame) {
  use_color_as_world_frame_ = use_color_as_world_frame;
  set_up_ = false;
}

bool RosTopicDepthCamera::UpdateImage(bool synchronized) {
  // std::cout << "Getting Depth\n";
  if (!set_up_) {
    std::cerr << "Set up real sense depth camera " << name_ << " first"
              << std::endl;
    return false;
  }

  if (!realsense_.got_frames_) {
    // std::cerr << "No depth frame received yet" << std::endl;
    return false;
  }

  // Get frameset and copy data to image
  // image_ = realsense_.topic_depth_ptr_->image; 
  cv::Mat{realsense_.topic_depth_ptr_->image}.copyTo(image_);
  // realsense_.UpdateCapture(realsense_id_, synchronized);
  // cv::Mat{cv::Size{intrinsics_.width, intrinsics_.height}, CV_16UC1,
  //         (void *)realsense_.frameset().get_depth_frame().get_data(),
  //         cv::Mat::AUTO_STEP}
  //     .copyTo(image_);

  SaveImageIfDesired();
  return true;
}

bool RosTopicDepthCamera::use_color_as_world_frame() const {
  return use_color_as_world_frame_;
}

const Transform3fA *RosTopicDepthCamera::color2depth_pose() const {
  return realsense_.color2depth_pose();
}

const Transform3fA *RosTopicDepthCamera::depth2color_pose() const {
  return realsense_.depth2color_pose();
}

bool RosTopicDepthCamera::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  ReadOptionalValueFromYaml(fs, "camera2world_pose", &camera2world_pose_);
  ReadOptionalValueFromYaml(fs, "save_directory", &save_directory_);
  ReadOptionalValueFromYaml(fs, "save_index", &save_index_);
  ReadOptionalValueFromYaml(fs, "save_image_type", &save_image_type_);
  ReadOptionalValueFromYaml(fs, "save_images", &save_images_);
  ReadOptionalValueFromYaml(fs, "use_color_as_world_frame",
                            &use_color_as_world_frame_);
  fs.release();

  // Process parameters
  if (save_directory_.is_relative())
    save_directory_ = metafile_path_.parent_path() / save_directory_;
  world2camera_pose_ = camera2world_pose_.inverse();
  return true;
}

void RosTopicDepthCamera::GetIntrinsics() {
  // const rs2_intrinsics intrinsics = realsense_.profile()
  //                                       .get_stream(RS2_STREAM_DEPTH)
  //                                       .as<rs2::video_stream_profile>()
  //                                       .get_intrinsics();
  // intrinsics_.fu = intrinsics.fx;
  // intrinsics_.fv = intrinsics.fy;
  // intrinsics_.ppu = intrinsics.ppx;
  // intrinsics_.ppv = intrinsics.ppy;
  // intrinsics_.width = intrinsics.width;
  // intrinsics_.height = intrinsics.height;
  // depth_scale_ = realsense_.profile()
  //                    .get_device()
  //                    .first<rs2::depth_sensor>()
  //                    .get_depth_scale();
}

}  // namespace icg
