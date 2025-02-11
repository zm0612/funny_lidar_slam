//
// Created by Zhang Zhimeng on 22-10-27.
//
#include "slam/system.h"

#include <pcl_conversions/pcl_conversions.h>

#include <thread>
#include <utility>

#include "common/constant_variable.h"
#include "common/pointcloud_utility.h"
#include "common/ros_utility.h"
#include "common/sensor_data_utility.h"
#include "lidar/lidar_point_type.h"
#include "optimization/g2o/g2o_optimizer_header.h"
#include "slam/config_parameters.h"
#include "slam/frontend.h"
#include "slam/localization.h"
#include "slam/loop_closure.h"
#include "slam/preprocessing.h"
#include "slam/split_map.h"

System::System(const rclcpp::NodeOptions& options)
    : Node("funny_lidar_slam", options) {
  InitConfigParameters();
  InitLidarModel();
  InitPublisher();
  InitSubscriber();
  this->tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  imu_data_searcher_ptr_ = std::make_shared<IMUDataSearcher>(
      ConfigParameters::Instance().imu_data_searcher_buffer_size_);

  if (slam_mode_ == SLAM_MODE::MAPPING) {
    InitMappingMode();
    LOG(INFO) << "\033[1;32m----> System Started. Mapping Mode \033[0m";
  } else if (slam_mode_ == SLAM_MODE::LOCALIZATION) {
    LOG(INFO) << "\033[1;32m----> System Started. Localization Mode\033[0m";
    InitLocalizationMode();
  } else {
    LOG(FATAL) << "Please select SLAM mode";
  }

  pre_processing_ptr_ = std::make_shared<PreProcessing>(this);
  pre_processing_thread_ptr_ =
      std::make_shared<std::thread>(&PreProcessing::Run, pre_processing_ptr_);
  //
  this->timer_cb_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  system_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(10),
                              std::bind(&System::Run, this), timer_cb_group_);
}

void System::InitMappingMode() {
  InitSaveMapServer();
  InitMappingPublisher();

  loop_closure_optimizer_ptr_ = std::make_shared<LoopClosureOptimizer>();

  front_end_ptr_ = std::make_shared<FrontEnd>(this);
  frontend_thread_ptr_ =
      std::make_shared<std::thread>(&FrontEnd::Run, front_end_ptr_.get());

  if (ConfigParameters::Instance().system_enable_loopclosure_) {
    loop_closure_ptr_ = std::make_shared<LoopClosure>(this);
    loop_closure_thread_ptr_ = std::make_shared<std::thread>(
        &LoopClosure::Run, loop_closure_ptr_.get());
  }

  if (ConfigParameters::Instance().system_enable_visualize_global_map_) {
    visualize_global_map_thread_ptr_ =
        std::make_shared<std::thread>(&System::VisualizeGlobalMap, this);
  }
}

void System::InitLocalizationMode() {
  InitLocalizationPublisher();
  localization_ptr_ = std::make_shared<Localization>(this);
  localization_thread_ptr_ = std::make_shared<std::thread>(
      &Localization::Run, localization_ptr_.get());
}

System::~System() {
  cv_frontend_.notify_one();
  if (frontend_thread_ptr_ && frontend_thread_ptr_->joinable()) {
    frontend_thread_ptr_->join();
  }
  if (loop_closure_thread_ptr_ && loop_closure_thread_ptr_->joinable()) {
    loop_closure_thread_ptr_->join();
  }
  if (visualize_global_map_thread_ptr_ &&
      visualize_global_map_thread_ptr_->joinable()) {
    visualize_global_map_thread_ptr_->join();
  }
  cv_localization_.notify_one();
  if (localization_thread_ptr_ && localization_thread_ptr_->joinable()) {
    localization_thread_ptr_->join();
  }
  cv_preprocessing_.notify_one();
  if (pre_processing_thread_ptr_ && pre_processing_thread_ptr_->joinable()) {
    pre_processing_thread_ptr_->join();
  }
}

void System::InitLidarModel() {
  if (ConfigParameters::Instance().lidar_sensor_type_ == "None") {
    LidarModel::Instance(ConfigParameters::Instance().lidar_sensor_type_);
    int lidar_horizon_scan = ConfigParameters::Instance().lidar_horizon_scan_;
    LidarModel::Instance()->horizon_scan_num_ = lidar_horizon_scan;
    LidarModel::Instance()->vertical_scan_num_ =
        ConfigParameters::Instance().lidar_scan_;
    LidarModel::Instance()->h_res_ =
        Degree2Radian(360.0f / static_cast<float>(lidar_horizon_scan));
    LidarModel::Instance()->v_res_ = static_cast<float>(
        Degree2Radian(ConfigParameters::Instance().lidar_vertical_resolution_));
    LidarModel::Instance()->lower_angle_ = static_cast<float>(
        Degree2Radian(ConfigParameters::Instance().lidar_lower_angle_));
  } else {
    LidarModel::Instance(ConfigParameters::Instance().lidar_sensor_type_);
  }
}

void System::InitConfigParameters() {
  ConfigParameters& config = ConfigParameters::Instance();

  // sensor topic name
  util::param(this, "sensor_topic.lidar_topic", config.lidar_topic_,
              config.lidar_topic_);

  util::param(this, "sensor_topic.imu_topic", config.imu_topic_,
              config.imu_topic_);
  int temp = 0;
  util::param(this, "slam_mode", temp, temp);
  slam_mode_ = static_cast<SLAM_MODE>(temp);

  // lidar config parameters
  util::param(this, "lidar.lidar_sensor_type", config.lidar_sensor_type_,
              StringEmpty);
  util::param(this, "lidar.lidar_point_jump_span",
              config.lidar_point_jump_span_, IntNaN);
  util::param(this, "lidar.lidar_scan", config.lidar_scan_, IntNaN);
  util::param(this, "lidar.lidar_lower_angle", config.lidar_lower_angle_,
              DoubleNaN);
  util::param(this, "lidar.lidar_horizon_scan", config.lidar_horizon_scan_,
              IntNaN);
  util::param(this, "lidar.lidar_vertical_resolution",
              config.lidar_vertical_resolution_, DoubleNaN);
  util::param(this, "lidar.lidar_use_min_distance", config.lidar_use_min_dist_,
              FloatNaN);
  util::param(this, "lidar.lidar_use_max_distance", config.lidar_use_max_dist_,
              FloatNaN);
  util::param(this, "lidar.lidar_point_time_scale",
              config.lidar_point_time_scale_, DoubleNaN);
  util::param(this, "lidar.lidar_rotation_noise_std",
              config.lidar_rotation_noise_std_, DoubleNaN);
  util::param(this, "lidar.lidar_position_noise_std",
              config.lidar_position_noise_std_, DoubleNaN);

  // imu config parameters
  util::param(this, "imu.has_orientation", config.imu_has_orientation_, false);
  util::param(this, "imu.init_acc_bias", config.imu_init_acc_bias_, DoubleNaN);
  util::param(this, "imu.init_gyro_bias", config.imu_init_gyro_bias_,
              DoubleNaN);
  util::param(this, "imu.acc_noise_std", config.imu_acc_noise_std_, DoubleNaN);
  util::param(this, "imu.gyro_noise_std", config.imu_gyro_noise_std_,
              DoubleNaN);
  util::param(this, "imu.acc_rw_noise_std", config.imu_acc_rw_noise_std_,
              DoubleNaN);
  util::param(this, "imu.gyro_rw_noise_std", config.imu_gyro_rw_noise_std_,
              DoubleNaN);
  util::param(this, "imu.data_searcher_buffer_size",
              config.imu_data_searcher_buffer_size_, IntNaN);

  // gravity
  util::param(this, "gravity", config.gravity_norm, DoubleNaN);

  // calibration parameters
  std::vector<double> lidar_to_imu = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                      0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  util::param_vector(this, "calibration.lidar_to_imu", lidar_to_imu,
                     lidar_to_imu);
  config.calibration_lidar_to_imu_ =
      Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(
          lidar_to_imu.data());

  // frontend config parameters
  util::param(this, "frontend.registration_and_searcher_mode",
              config.registration_and_searcher_mode_, StringEmpty);
  util::param(this, "frontend.feature.corner_thres",
              config.loam_feature_corner_thres_, FloatNaN);
  util::param(this, "frontend.feature.planar_thres",
              config.loam_feature_planar_thres_, FloatNaN);
  util::param(this, "frontend.feature.planar_voxel_filter_size",
              config.loam_feature_planar_voxel_filter_size_, FloatNaN);
  util::param(this, "frontend.feature.corner_voxel_filter_size",
              config.loam_feature_corner_voxel_filter_size_, FloatNaN);
  util::param(this, "frontend.registration.line_ratio_thres",
              config.registration_line_ratio_thres_, FloatNaN);
  util::param(this, "frontend.registration.point_search_thres",
              config.registration_point_search_thres_, FloatNaN);
  util::param(this, "frontend.registration.point_to_planar_thres",
              config.registration_point_to_planar_thres_, FloatNaN);
  util::param(this, "frontend.registration.local_planar_map_size",
              config.registration_local_planar_map_size_, IntNaN);
  util::param(this, "frontend.registration.local_corner_map_size",
              config.registration_local_corner_map_size_, IntNaN);
  util::param(this, "frontend.registration.keyframe_delta_distance",
              config.registration_keyframe_delta_dist_, DoubleNaN);
  util::param(this, "frontend.registration.keyframe_delta_rotation",
              config.registration_keyframe_delta_rotation_, DoubleNaN);
  util::param(this, "frontend.registration.rotation_converge_thres",
              config.registration_rotation_converge_thres_, FloatNaN);
  util::param(this, "frontend.registration.position_converge_thres",
              config.registration_position_converge_thres_, FloatNaN);
  util::param(this, "frontend.registration.local_corner_voxel_filter_size",
              config.registration_local_corner_filter_size_, FloatNaN);
  util::param(this, "frontend.registration.local_planar_voxel_filter_size",
              config.registration_local_planar_filter_size_, FloatNaN);
  util::param(this, "frontend.registration.local_map_size",
              config.registration_local_map_size_, IntNaN);
  util::param(this, "frontend.registration.local_map_cloud_filter_size",
              config.registration_local_map_cloud_filter_size_, FloatNaN);
  util::param(this, "frontend.registration.source_cloud_filter_size",
              config.registration_source_cloud_filter_size_, FloatNaN);
  util::param(this, "frontend.registration.optimization_iter_num",
              config.registration_opti_iter_num_, IntNaN);
  util::param(this, "frontend.registration.ndt_voxel_size",
              config.registration_ndt_voxel_size_, DoubleNaN);
  util::param(this, "frontend.registration.ndt_outlier_threshold",
              config.registration_ndt_outlier_threshold_, DoubleNaN);
  util::param(this, "frontend.registration.ndt_min_points_in_voxel",
              config.registration_ndt_min_points_in_voxel_, IntNaN);
  util::param(this, "frontend.registration.ndt_max_points_in_voxel",
              config.registration_ndt_max_points_in_voxel_, IntNaN);
  util::param(this, "frontend.registration.ndt_min_effective_pts",
              config.registration_ndt_min_effective_pts_, IntNaN);
  util::param(this, "frontend.registration.ndt_capacity",
              config.registration_ndt_capacity_, IntNaN);
  util::param(this, "frontend.fusion_method", config.fusion_method_,
              StringEmpty);
  util::param(this, "frontend.fusion_opti_iters",
              config.frontend_fusion_opti_iters_, IntNaN);

  // system config parameters
  util::param(this, "system.keyframe_delta_distance",
              config.system_keyframe_delta_dist_, DoubleNaN);
  util::param(this, "system.keyframe_delta_rotation",
              config.system_keyframe_delta_rotation_, DoubleNaN);
  util::param(this, "system.enable_loopclosure",
              config.system_enable_loopclosure_, false);
  util::param(this, "system.enable_visualize_global_map",
              config.system_enable_visualize_global_map_, false);
  util::param(this, "system.global_map_visualization_resolution",
              config.system_global_map_visualization_resolution_, FloatNaN);

  // split map
  util::param(this, "system.tile_map_grid_size", config.tile_map_grid_size_,
              DoubleNaN);

  // loopclosure config parameters
  util::param(this, "loopclosure.skip_near_loopclosure_threshold",
              config.lc_skip_near_loopclosure_threshold_, IntNaN);
  util::param(this, "loopclosure.skip_near_keyframe_threshold",
              config.lc_skip_near_keyframe_threshold_, IntNaN);
  util::param(this, "loopclosure.candidate_local_map_left_range",
              config.lc_candidate_local_map_left_range_, IntNaN);
  util::param(this, "loopclosure.candidate_local_map_right_range",
              config.lc_candidate_local_map_right_range_, IntNaN);
  util::param(this, "loopclosure.loopclosure_local_map_left_range",
              config.lc_loopclosure_local_map_left_range_, IntNaN);
  util::param(this, "loopclosure.near_neighbor_distance_threshold",
              config.lc_near_neighbor_distance_threshold_, DoubleNaN);
  util::param(this, "loopclosure.registration_converge_threshold",
              config.lc_registration_converge_threshold_, FloatNaN);
}

void System::InitLocalizationPublisher() {
  localization_path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("/localization_path", 5);
  localization_local_cloud_map_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/localization_local_map",
                                                      5);
  localization_global_cloud_map_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
          "/localization_global_map",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  localization_current_lidar_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
          "/localization_current_lidar", 5);
}

void System::InitMappingPublisher() {
  mapping_keyframe_path_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/keyframe_cloud_path",
                                                      5);
  mapping_keyframe_path_pub_ =
      create_publisher<nav_msgs::msg::Path>("/keyframe_path", 5);
  mapping_curr_corner_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/corner_cloud", 5);
  mapping_curr_planer_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/planer_cloud", 5);
  mapping_curr_keyframe_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/keyframe_cloud", 2);
  mapping_global_map_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/global_map_cloud", 2);
}

void System::InitPublisher() {
  mapping_curr_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/frame_cloud", 100);
}

void System::InitSubscriber() {
  this->lidar_cb_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto lidar_sub_opt = rclcpp::SubscriptionOptions();
  lidar_sub_opt.callback_group = this->lidar_cb_group_;
  if (LidarModel::Instance()->lidar_sensor_type_ ==
      LidarModel::LidarSensorType::LIVOX_AVIA) {
    lidar_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        ConfigParameters::Instance().lidar_topic_,
        rclcpp::QoS(10).best_effort().keep_last(1),
        std::bind(&System::LidarLivoxMsgCallBack, this, std::placeholders::_1),
        lidar_sub_opt);
  } else {
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        ConfigParameters::Instance().lidar_topic_,
        rclcpp::QoS(10).best_effort().keep_last(1),
        std::bind(&System::LidarStandardMsgCallBack, this,
                  std::placeholders::_1),
        lidar_sub_opt);
  }

  this->imu_cb_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto imu_sub_opt = rclcpp::SubscriptionOptions();
  imu_sub_opt.callback_group = this->imu_cb_group_;
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      ConfigParameters::Instance().imu_topic_,
      rclcpp::QoS(400).best_effort().keep_last(1),
      std::bind(&System::ImuMsgCallBack, this, std::placeholders::_1),
      imu_sub_opt);

  if (slam_mode_ == SLAM_MODE::LOCALIZATION) {
    this->init_pose_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    auto init_sub_opt = rclcpp::SubscriptionOptions();
    init_sub_opt.callback_group = this->init_pose_cb_group_;
    localization_init_pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", rclcpp::QoS(1).best_effort().keep_last(1),
        std::bind(&System::LocalizationInitPoseMsgCallBack, this,
                  std::placeholders::_1),
        init_sub_opt);
  }
}

void System::InitSaveMapServer() {
  this->save_map_cb_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  save_map_server_ = this->create_service<funny_lidar_slam::srv::SaveMap>(
      "save_map",
      std::bind(&System::SaveMap, this, std::placeholders::_1,
                std::placeholders::_2),
      qos_profile, save_map_cb_group_);
}

bool System::SaveMap(
    std::shared_ptr<funny_lidar_slam::srv::SaveMap::Request> req,
    std::shared_ptr<funny_lidar_slam::srv::SaveMap::Response> res) {
  std::lock_guard<std::mutex> lg(mutex_keyframes_);

  res->map_path = StringEmpty;
  res->map_size = 0u;
  res->save_map_succeed = false;

  if (keyframes_.empty()) {
    return false;
  }

  PCLPointCloudXYZI::Ptr map_cloud(new PCLPointCloudXYZI);
  for (const auto& keyframe : keyframes_) {
    auto cloud = VoxelGridCloud(*keyframe->LoadOrderedCloud(), 0.3f);
    *map_cloud += TransformPointCloud(*cloud, keyframe->pose_);
  }

  map_cloud = VoxelGridCloud(*map_cloud, 0.3);

  if (!map_cloud->empty()) {
    std::string map_path;

    if (req->map_path.empty()) {
      map_path = std::string(PROJECT_SOURCE_DIR) + "/data/map.pcd";
    } else {
      map_path = req->map_path;
    }

    pcl::io::savePCDFileBinary(map_path, *map_cloud);
    res->map_path = map_path;
    res->map_size = static_cast<int64_t>(map_cloud->size());
    res->save_map_succeed = true;

    // TODO: 根据传入的路径保存
    if (req->split_map == true) {
      const SplitMap split_map;
      split_map.Split(*map_cloud);
    }
  }

  return true;
}

void System::ImuMsgCallBack(
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu_ptr) {
  static Vec3d init_mean_acc = Vec3d::Zero();
  static Vec3d last_angular_velocity;
  static Eigen::Quaterniond last_orientation;
  static TimeStampUs last_timestamp;

  IMUData imu_data;
  imu_data.timestamp_ = RosTimeToUs(imu_ptr->header);
  imu_data.angular_velocity_ = RosVec3dToEigen(imu_ptr->angular_velocity);
  imu_data.linear_acceleration_ = RosVec3dToEigen(imu_ptr->linear_acceleration);

  if (ConfigParameters::Instance().fusion_method_ == kFusionLooseCoupling) {
    if (!has_imu_init_.load()) {
      if (ConfigParameters::Instance().imu_has_orientation_) {
        imu_data.orientation_ = RosQuaternionToEigen(imu_ptr->orientation);
      } else {
        imu_data.orientation_ = Eigen::Quaterniond::Identity();
        last_angular_velocity = imu_data.angular_velocity_;
        last_orientation = Eigen::Quaterniond::Identity();
        last_timestamp = imu_data.timestamp_;
      }
      has_imu_init_.store(true);
      imu_data_searcher_ptr_->CacheData(imu_data);
      return;
    }
  } else {
    if (!has_imu_init_.load()) {
      has_imu_init_.store(InitIMU(imu_data, init_mean_acc));

      if (has_imu_init_.load()) {
        if (ConfigParameters::Instance().imu_has_orientation_) {
          imu_data.orientation_ = RosQuaternionToEigen(imu_ptr->orientation);
        } else {
          imu_data.orientation_ = Eigen::Quaterniond::Identity();
          last_angular_velocity = imu_data.angular_velocity_;
          last_orientation = Eigen::Quaterniond::Identity();
          last_timestamp = imu_data.timestamp_;
        }

        imu_data.linear_acceleration_ =
            imu_data.linear_acceleration_ *
            ConfigParameters::Instance().gravity_norm / init_mean_acc.norm();

        imu_data_searcher_ptr_->CacheData(imu_data);
      }
      return;
    }

    imu_data.linear_acceleration_ = imu_data.linear_acceleration_ *
                                    ConfigParameters::Instance().gravity_norm /
                                    init_mean_acc.norm();
  }

  if (ConfigParameters::Instance().imu_has_orientation_) {
    imu_data.orientation_ = RosQuaternionToEigen(imu_ptr->orientation);
  } else {
    const Eigen::Quaterniond delta_q(
        SO3Exp((last_angular_velocity + imu_data.angular_velocity_) * 0.5 *
               (imu_data.timestamp_ - last_timestamp) * kMicroseconds2Seconds));
    imu_data.orientation_ = last_orientation * delta_q;

    last_timestamp = imu_data.timestamp_;
    last_orientation = imu_data.orientation_;
    last_angular_velocity = imu_data.angular_velocity_;
  }

  imu_data_searcher_ptr_->CacheData(imu_data);
}

bool System::InitIMU(const IMUData& imu_data, Vec3d& init_mean_acc) {
  static Vec3d mean_acc = Vec3d::Zero();
  static Vec3d mean_gyro = Vec3d::Zero();
  static Vec3d cov_acc = Vec3d::Zero();
  static Vec3d cov_gyro = Vec3d::Zero();
  static int N = 0;

  if (N == 0) {
    mean_acc = imu_data.linear_acceleration_;
    mean_gyro = imu_data.angular_velocity_;
    N = 1;
  } else {
    const auto& acc = imu_data.linear_acceleration_;
    const auto& gyro = imu_data.angular_velocity_;

    mean_acc += (acc - mean_acc) / N;
    mean_gyro += (gyro - mean_gyro) / N;

    cov_acc =
        cov_acc * (N - 1.0) / N +
        (acc - mean_acc).cwiseProduct(acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyro =
        cov_gyro * (N - 1.0) / N +
        (gyro - mean_gyro).cwiseProduct(gyro - mean_gyro) * (N - 1.0) / (N * N);

    N++;
  }

  init_mean_acc = mean_acc;

  if (N > 300) {
    N = 0;
    mean_acc = Vec3d::Zero();
    mean_gyro = Vec3d::Zero();
    cov_acc = Vec3d::Zero();
    cov_gyro = Vec3d::Zero();

    LOG(WARNING) << "IMU movement acceleration is too large, Reinitialize!";
    return false;
  }

  if (N > 200 && cov_acc.norm() < 0.05 && cov_gyro.norm() < 0.01) {
    ConfigParameters::Instance().gravity_vector_ =
        -mean_acc / mean_acc.norm() * ConfigParameters::Instance().gravity_norm;
    return true;
  }

  return false;
}

void System::LidarStandardMsgCallBack(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_ros_ptr) {
  if (has_imu_init_.load()) {
    std::lock_guard<std::mutex> lk(mutex_raw_cloud_deque_);

    raw_cloud_deque_.push_back(cloud_ros_ptr);
    cv_preprocessing_.notify_one();
  }
}

void System::LocalizationInitPoseMsgCallBack(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg) {
  Eigen::Quaterniond q = RosQuaternionToEigen(msg->pose.pose.orientation);
  Vec3d t = RosPoint3dToEigen(msg->pose.pose.position);
  Mat4d pose = Mat4d::Identity();
  pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  pose.block<3, 1>(0, 3) = t;

  localization_ptr_->SetInitPose(pose);
}

void System::LidarLivoxMsgCallBack(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg) {
  using PointField = sensor_msgs::msg::PointField;

  static sensor_msgs::msg::PointCloud2 cloud_ros;
  static bool init_ros_cloud_type = false;

  if (!init_ros_cloud_type) {
    const auto AddField = [&](const std::string& name, uint32_t offset,
                              uint8_t datatype) {
      sensor_msgs::msg::PointField field;
      field.count = 1u;
      field.name = name;
      field.offset = offset;
      field.datatype = datatype;
      cloud_ros.fields.push_back(field);
    };

    AddField("x", 0, PointField::FLOAT32);
    AddField("y", cloud_ros.fields.back().offset + sizeof(float),
             PointField::FLOAT32);
    AddField("z", cloud_ros.fields.back().offset + sizeof(float),
             PointField::FLOAT32);
    AddField("intensity", cloud_ros.fields.back().offset + sizeof(float),
             PointField::FLOAT32);
    AddField("time", cloud_ros.fields.back().offset + sizeof(uint32_t),
             PointField::UINT32);
    AddField("line", cloud_ros.fields.back().offset + sizeof(uint8_t),
             PointField::UINT8);
    AddField("tag", cloud_ros.fields.back().offset + sizeof(uint8_t),
             PointField::UINT8);
    cloud_ros.is_bigendian = false;
    cloud_ros.point_step =
        sizeof(float) * 4u + sizeof(uint32_t) + sizeof(uint8_t) * 2u;
    cloud_ros.is_dense = true;

    init_ros_cloud_type = true;
  }

  if (has_imu_init_.load()) {
    size_t point_size = msg->point_num;

    cloud_ros.header = msg->header;
    cloud_ros.width = point_size;
    cloud_ros.height = 1;
    cloud_ros.row_step = point_size * cloud_ros.point_step;
    cloud_ros.data.resize(point_size * cloud_ros.point_step);

    pcl::PointCloud<PointXYZIRT>::Ptr cloud_in_ptr(
        new pcl::PointCloud<PointXYZIRT>());
    cloud_in_ptr->reserve(point_size);

    unsigned char* ptr = cloud_ros.data.data();
    for (size_t i = 0; i < point_size; ++i) {
      *reinterpret_cast<float*>(ptr + cloud_ros.fields[0].offset) =
          msg->points[i].x;
      *reinterpret_cast<float*>(ptr + cloud_ros.fields[1].offset) =
          msg->points[i].y;
      *reinterpret_cast<float*>(ptr + cloud_ros.fields[2].offset) =
          msg->points[i].z;
      *reinterpret_cast<float*>(ptr + cloud_ros.fields[3].offset) =
          static_cast<float>(msg->points[i].reflectivity);
      *reinterpret_cast<uint32_t*>(ptr + cloud_ros.fields[4].offset) =
          msg->points[i].offset_time;
      *reinterpret_cast<uint8_t*>(ptr + cloud_ros.fields[5].offset) =
          msg->points[i].line;
      *reinterpret_cast<uint8_t*>(ptr + cloud_ros.fields[6].offset) =
          msg->points[i].tag;

      ptr += cloud_ros.point_step;
    }

    sensor_msgs::msg::PointCloud2::SharedPtr cloud_ros_ptr(
        new sensor_msgs::msg::PointCloud2);
    *cloud_ros_ptr = cloud_ros;

    std::lock_guard<std::mutex> lk(mutex_raw_cloud_deque_);
    raw_cloud_deque_.push_back(cloud_ros_ptr);
    cv_preprocessing_.notify_one();
  }
}

void System::Run() {
  if (slam_mode_ == SLAM_MODE::MAPPING) {
    RunMapping();
  } else if (slam_mode_ == SLAM_MODE::LOCALIZATION) {
    RunLocalization();
  } else {
    LOG(FATAL) << "SLAM Mode Error!";
  }
}

void System::RunMapping() {
  if (ProcessMappingFrameCache()) {
    PublishMappingKeyFramePath();
  }

  if (HasLoopClosure()) {
    PerformLoopclosureOptimization();
    PublishMappingKeyFramePath();
  }
}

void System::RunLocalization() {
  // publish localization path
  if (ProcessLocalizationResultCache()) {
    PublishLocalizationPath();

    const auto current_lidar_cloud =
        localization_ptr_->GetCurrentLidarCloudMap();
    PublishRosCloud(localization_current_lidar_cloud_pub_,
                    current_lidar_cloud.makeShared());
  }

  // publish local cloud map
  if (localization_ptr_->NeedUpdateLocalMapVisualization()) {
    const auto local_cloud_map = localization_ptr_->GetLocalCloudMap();
    PublishRosCloud(localization_local_cloud_map_pub_,
                    local_cloud_map.makeShared());
  }

  // publish global cloud map
  static bool first_global_map_visualization = true;
  if (first_global_map_visualization &&
      localization_global_cloud_map_pub_->get_subscription_count() > 0) {
    const auto global_cloud_map = localization_ptr_->GetGlobalCloudMap();

    if (!global_cloud_map.empty()) {
      PublishRosCloud(localization_global_cloud_map_pub_,
                      global_cloud_map.makeShared());
      first_global_map_visualization = false;
    }
  }
}

bool System::HasLoopClosure() {
  std::lock_guard<std::mutex> lg(mutex_loopclosure_results_);
  return !loopclosure_result_deque_.empty();
}

bool System::ProcessLocalizationResultCache() {
  NavStateData::Ptr nav_state_data;

  {
    std::lock_guard<std::mutex> lg(mutex_localization_results_deque_);

    if (localization_results_deque_.empty()) {
      return false;
    }

    nav_state_data = localization_results_deque_.front();
    localization_results_deque_.pop_front();
  }

  const Mat4d pose = nav_state_data->Pose();
  PublishTF(pose, nav_state_data->timestamp_);

  const Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
  const Vec3d& t = pose.block<3, 1>(0, 3);
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.pose.orientation.x = q.x();
  pose_stamped.pose.orientation.y = q.y();
  pose_stamped.pose.orientation.z = q.z();
  pose_stamped.pose.orientation.w = q.w();
  pose_stamped.pose.position.x = t.x();
  pose_stamped.pose.position.y = t.y();
  pose_stamped.pose.position.z = t.z();
  pose_stamped.header.frame_id = kRosMapFrameID;
  pose_stamped.header.stamp = this->get_clock()->now();
  localization_path_.poses.emplace_back(std::move(pose_stamped));

  return true;
}

bool System::ProcessMappingFrameCache() {
  Frame::Ptr frame;
  {
    std::lock_guard<std::mutex> lg(mutex_frame_temp_deque_);

    if (frame_temp_deque_.empty()) {
      return false;
    }

    frame = frame_temp_deque_.front();
    frame_temp_deque_.pop_front();
  }

  // Used to determine whether keyframe needs to be added
  static Mat4d accumulated_pose = Mat4d::Identity();
  Mat4d curr_pose;
  accumulated_pose *= frame->delta_pose_;

  {
    std::lock_guard<std::mutex> lg(mutex_keyframes_);
    if (IsKeyFrame(accumulated_pose)) {
      KeyFrame::Ptr keyframe = std::make_shared<KeyFrame>();
      keyframe->timestamp_ = frame->timestamp_;
      keyframe->cloud_cluster_ptr_ = frame->cloud_cluster_ptr_;

      if (keyframes_.empty()) {
        keyframe->id_ = 0;
        keyframe->pose_ = accumulated_pose;
        loop_closure_optimizer_ptr_->AddVertex(keyframe->pose_, keyframe->id_,
                                               true);
      } else {
        keyframe->id_ = keyframes_.back()->id_ + 1;
        keyframe->pose_ = keyframes_.back()->pose_ * accumulated_pose;
        loop_closure_optimizer_ptr_->AddVertex(keyframe->pose_, keyframe->id_,
                                               false);
      }

      keyframes_.push_back(keyframe);

      if (keyframes_.size() >= 2u) {
        Mat6d info = Mat6d::Zero();
        info.diagonal() << 1.0, 1.0, 1.0, 100.0, 100.0, 100.0;
        const auto& last_keyframe = *(keyframes_.rbegin() + 1);
        const auto& curr_keyframe = *(keyframes_.rbegin());
        const Mat4d delta_pose =
            last_keyframe->pose_.inverse() * curr_keyframe->pose_;
        loop_closure_optimizer_ptr_->AddEdge(
            delta_pose, info, last_keyframe->id_, curr_keyframe->id_);
      }

      PublishMappingKeyFrameCloud(keyframe);

      keyframe->SaveAllCloud();
      accumulated_pose.setIdentity();  // reset for next keyframe judgement
    }

    curr_pose = keyframes_.back()->pose_ * accumulated_pose;
  }

  PublishMappingFrameCloud(frame, curr_pose);
  PublishTF(curr_pose, frame->timestamp_);

  // At this point, the pointcloud in the cloud cluster is no longer needed!
  frame->cloud_cluster_ptr_->Clear();

  return true;
}

void System::PerformLoopclosureOptimization() {
  LoopClosureResult loopclosure_result;
  {
    std::lock_guard<std::mutex> lg(mutex_loopclosure_results_);
    if (loopclosure_result_deque_.empty()) {
      return;
    } else {
      loopclosure_result = loopclosure_result_deque_.front();
      loopclosure_result_deque_.pop_front();
    }
  }

  Mat6d info = Mat6d::Zero();
  info.diagonal() << 1.0, 1.0, 1.0, 100.0, 100.0, 100.0;
  loop_closure_optimizer_ptr_->AddEdge(
      loopclosure_result.match_pose_, info,
      loopclosure_result.candidate_keyframe_id_,
      loopclosure_result.loopclosure_keyframe_id_);

  loop_closure_optimizer_ptr_->Optimize(15);

  {
    std::lock_guard<std::mutex> lg(mutex_keyframes_);

    for (auto& keyframe : keyframes_) {
      const auto id = keyframe->id_;
      keyframe->pose_ = loop_closure_optimizer_ptr_->GetVertexEstimate(id);
    }
  }

  need_update_global_map_visualization_.store(true);
}

void System::PublishLocalizationPath() {
  if (localization_path_pub_->get_subscription_count() > 0) {
    localization_path_.header.frame_id = kRosMapFrameID;
    localization_path_.header.stamp = this->get_clock()->now();
    localization_path_pub_->publish(localization_path_);
  }
}

void System::PublishMappingKeyFramePath() {
  if (mapping_keyframe_path_cloud_pub_->get_subscription_count() > 0) {
    std::lock_guard<std::mutex> lg(mutex_keyframes_);

    PCLPointCloudXYZI::Ptr path_cloud(new PCLPointCloudXYZI);
    for (const auto& keyframe : keyframes_) {
      const auto& pose = keyframe->pose_;

      PCLPointXYZI point;
      point.x = static_cast<float>(pose(0, 3));
      point.y = static_cast<float>(pose(1, 3));
      point.z = static_cast<float>(pose(2, 3));
      point.intensity = 0;
      path_cloud->emplace_back(point);
    }

    PublishRosCloud(mapping_keyframe_path_cloud_pub_, path_cloud->makeShared());
  }

  if (mapping_keyframe_path_pub_->get_subscription_count() > 0) {
    std::lock_guard<std::mutex> lg(mutex_keyframes_);

    nav_msgs::msg::Path path;
    for (const auto& keyframe : keyframes_) {
      const auto& pose = keyframe->pose_;

      const Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
      const Vec3d& t = pose.template block<3, 1>(0, 3);

      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.pose.orientation.x = q.x();
      pose_stamped.pose.orientation.y = q.y();
      pose_stamped.pose.orientation.z = q.z();
      pose_stamped.pose.orientation.w = q.w();
      pose_stamped.pose.position.x = t.x();
      pose_stamped.pose.position.y = t.y();
      pose_stamped.pose.position.z = t.z();
      pose_stamped.header.frame_id = kRosMapFrameID;

      path.poses.emplace_back(std::move(pose_stamped));
    }

    path.header.frame_id = kRosMapFrameID;
    path.header.stamp = this->get_clock()->now();
    mapping_keyframe_path_pub_->publish(path);
  }
}

bool System::IsKeyFrame(const Mat4d& delta_pose) const {
  if (keyframes_.empty()) {
    return true;
  }

  const double delta_t = delta_pose.block<3, 1>(0, 3).norm();
  const Mat3d R_delta = delta_pose.block<3, 3>(0, 0);
  const Vec3d delta_euler = RotationMatrixToRPY(R_delta).cwiseAbs();

  if (delta_t > ConfigParameters::Instance().system_keyframe_delta_dist_ ||
      delta_euler.x() >
          ConfigParameters::Instance().system_keyframe_delta_rotation_ ||
      delta_euler.y() >
          ConfigParameters::Instance().system_keyframe_delta_rotation_ ||
      delta_euler.z() >
          ConfigParameters::Instance().system_keyframe_delta_rotation_) {
    return true;
  } else {
    return false;
  }
}

void System::PublishMappingKeyFrameCloud(const KeyFrame::Ptr& keyframe) {
  CHECK_NOTNULL(keyframe);
  CHECK_NOTNULL(keyframe->cloud_cluster_ptr_);
  if (keyframe->cloud_cluster_ptr_->ordered_cloud_.empty()) {
    return;
  }

  PCLPointCloudXYZI::Ptr cloud_trans(new PCLPointCloudXYZI);

  *cloud_trans = keyframe->cloud_cluster_ptr_->ordered_cloud_;
  cloud_trans = TransformPointCloud(cloud_trans, keyframe->pose_);
  PublishRosCloud(mapping_curr_keyframe_cloud_pub_, cloud_trans);
}

void System::PublishMappingFrameCloud(const Frame::Ptr& frame,
                                      const Mat4d& pose) {
  CHECK_NOTNULL(frame);

  PCLPointCloudXYZI::Ptr cloud_trans(new PCLPointCloudXYZI);

  if (!frame->cloud_cluster_ptr_->ordered_cloud_.empty()) {
    cloud_trans =
        VoxelGridCloud(frame->cloud_cluster_ptr_->ordered_cloud_, 0.5);
    cloud_trans = TransformPointCloud(cloud_trans, pose);
    PublishRosCloud(mapping_curr_cloud_pub_, cloud_trans);
  }

  if (!frame->cloud_cluster_ptr_->corner_cloud_.empty()) {
    *cloud_trans = frame->cloud_cluster_ptr_->corner_cloud_;
    cloud_trans = TransformPointCloud(cloud_trans, pose);
    PublishRosCloud(mapping_curr_corner_cloud_pub_, cloud_trans);
  }

  if (!frame->cloud_cluster_ptr_->planar_cloud_.empty()) {
    *cloud_trans = frame->cloud_cluster_ptr_->planar_cloud_;
    cloud_trans = TransformPointCloud(cloud_trans, pose);
    PublishRosCloud(mapping_curr_planer_cloud_pub_, cloud_trans);
  }
}

void System::PublishTF(const Mat4d& pose, TimeStampUs timestamp) {
  tf_broadcaster_->sendTransform(eigen2Transform(
      pose.block<3, 3>(0, 0), pose.block<3, 1>(0, 3), kRosMapFrameID,
      kRosLidarFrameID, static_cast<double>(timestamp) * 1e3));
}

void System::VisualizeGlobalMap() {
  CHECK(slam_mode_ == SLAM_MODE::MAPPING)
      << "VisualizeGlobalMap can only be used in mapping mode!";

  LOG(INFO) << "\033[1;32m----> Global Map Visualization Started.\033[0m";
  PCLPointCloudXYZI::Ptr global_map(new PCLPointCloudXYZI);
  KeyFrame::ID last_frame_id = -1;

  float voxel_filter_size =
      ConfigParameters::Instance().system_global_map_visualization_resolution_;

  rclcpp::Rate rate(0.5);
  while (rclcpp::ok()) {
    rate.sleep();

    if (mapping_global_map_cloud_pub_->get_subscription_count() == 0) {
      continue;
    }

    if (need_update_global_map_visualization_.load()) {
      need_update_global_map_visualization_.store(false);
      global_map->clear();
      last_frame_id = -1;
    }

    std::vector<KeyFrame::Ptr> keyframes;
    {
      std::lock_guard<std::mutex> lg(mutex_keyframes_);

      if (keyframes_.empty() || (last_frame_id + 1 >= keyframes_.back()->id_)) {
        continue;
      }

      for (int i = last_frame_id + 1; i <= keyframes_.back()->id_; ++i) {
        keyframes.push_back(keyframes_[i]);
      }
      last_frame_id = keyframes_.back()->id_;
    }

    if (!keyframes.empty()) {
      PCLPointCloudXYZI temp_local_cloud;
      for (const auto& keyframe : keyframes) {
        auto cloud = keyframe->LoadOrderedCloud();
        temp_local_cloud += *TransformPointCloud(
            VoxelGridCloud(*cloud, voxel_filter_size), keyframe->pose_);
      }

      *global_map += temp_local_cloud;
      global_map = VoxelGridCloud(*global_map, voxel_filter_size);

      PublishRosCloud(mapping_global_map_cloud_pub_, global_map);
    }
  }
}
