//
// Created by Zhang Zhimeng on 24-6-4.
//
#include "slam/localization.h"

#include <pcl/filters/crop_box.h>

#include "optimization/g2o/accel_bias_rw_edge.h"
#include "optimization/g2o/gyro_bias_rw_edge.h"
#include "optimization/g2o/position_edge.h"
#include "optimization/g2o/pre_integration_edge.h"
#include "optimization/g2o/prior_nav_state_edge.h"
#include "optimization/g2o/rotation_edge.h"
#include "registration/icp_optimized.h"
#include "registration/incremental_ndt.h"
#include "registration/loam_point_to_plane_ivox.h"
#include "registration/loam_point_to_plane_kdtree.h"
#include "slam/split_map.h"

Localization::Localization(System* system_ptr) : system_ptr_(system_ptr) {
  if (use_tile_map_) {
    tile_map_grid_size_ = ConfigParameters::Instance().tile_map_grid_size_;
    tile_map_grid_size_half_ = tile_map_grid_size_ * 0.5;
  }
  SetLidarPoseInformation();
  InitMatcher();
}

void Localization::SetInitPose(const Mat4d& pose) {
  std::lock_guard<std::mutex> lg(mutex_init_pose_optional_);
  init_pose_optional_ = pose;
  has_init_ = false;
}

void Localization::SetLidarPoseInformation() {
  const auto& lidar_rot_std =
      ConfigParameters::Instance().lidar_rotation_noise_std_;
  const auto& lidar_posi_std =
      ConfigParameters::Instance().lidar_position_noise_std_;

  lidar_pose_info_.setZero();
  lidar_pose_info_.block<3, 3>(0, 0) =
      Mat3d::Identity() * 1.0 / std::pow(lidar_rot_std, 2.0);
  lidar_pose_info_.block<3, 3>(3, 3) =
      Mat3d::Identity() * 1.0 / std::pow(lidar_posi_std, 2.0);
}

void Localization::InitMatcher() {
  if (ConfigParameters::Instance().registration_and_searcher_mode_ ==
      kIcpOptimized) {
    matcher_ = std::make_shared<IcpOptimized<double>>(
        ConfigParameters::Instance().registration_opti_iter_num_,
        ConfigParameters::Instance().registration_local_map_size_,
        static_cast<float>(ConfigParameters::Instance()
                               .registration_local_map_cloud_filter_size_),
        static_cast<float>(ConfigParameters::Instance()
                               .registration_source_cloud_filter_size_),
        static_cast<double>(
            ConfigParameters::Instance().registration_point_search_thres_),
        static_cast<double>(
            ConfigParameters::Instance().registration_position_converge_thres_),
        static_cast<double>(
            ConfigParameters::Instance().registration_rotation_converge_thres_),
        static_cast<double>(
            ConfigParameters::Instance().registration_keyframe_delta_rotation_),
        static_cast<double>(
            ConfigParameters::Instance().registration_keyframe_delta_dist_),
        true);
  } else if (ConfigParameters::Instance().registration_and_searcher_mode_ ==
             kIncrementalNDT) {
    matcher_ = std::make_shared<IncrementalNDT>(
        ConfigParameters::Instance().registration_ndt_voxel_size_,
        ConfigParameters::Instance().registration_ndt_outlier_threshold_,
        ConfigParameters::Instance().registration_source_cloud_filter_size_,
        static_cast<double>(
            ConfigParameters::Instance().registration_rotation_converge_thres_),
        static_cast<double>(
            ConfigParameters::Instance().registration_position_converge_thres_),
        ConfigParameters::Instance().registration_ndt_min_points_in_voxel_,
        ConfigParameters::Instance().registration_ndt_max_points_in_voxel_,
        ConfigParameters::Instance().registration_ndt_min_effective_pts_,
        ConfigParameters::Instance().registration_ndt_capacity_,
        ConfigParameters::Instance().registration_opti_iter_num_, true);
  } else if (ConfigParameters::Instance().registration_and_searcher_mode_ ==
             kPointToPlane_IVOX) {
    matcher_ = std::make_shared<LoamPointToPlaneIVOX<double>>(
        ConfigParameters::Instance().registration_point_to_planar_thres_,
        ConfigParameters::Instance().registration_position_converge_thres_,
        ConfigParameters::Instance().registration_rotation_converge_thres_,
        ConfigParameters::Instance().registration_opti_iter_num_, true);
  } else if (ConfigParameters::Instance().registration_and_searcher_mode_ ==
             kPointToPlane_KdTree) {
    matcher_ = std::make_shared<LoamPointToPlaneKdtree<double>>(
        ConfigParameters::Instance().registration_point_to_planar_thres_,
        ConfigParameters::Instance().registration_position_converge_thres_,
        ConfigParameters::Instance().registration_rotation_converge_thres_,
        ConfigParameters::Instance().registration_keyframe_delta_rotation_,
        ConfigParameters::Instance().registration_keyframe_delta_dist_,
        ConfigParameters::Instance().registration_local_map_size_,
        ConfigParameters::Instance().registration_local_map_cloud_filter_size_,
        ConfigParameters::Instance().registration_opti_iter_num_, true);
  } else {
    LOG(FATAL) << "Unsupported registration type! "
                  "You can choose one of the following: "
                  "IcpOptimized, kIncrementalNDT";
  }
}

void Localization::InitPreintegration() {
  PreIntegration::ConfigPara config_para;
  config_para.init_acc_bias_ =
      Vec3d(ConfigParameters::Instance().imu_init_acc_bias_,
            ConfigParameters::Instance().imu_init_acc_bias_,
            ConfigParameters::Instance().imu_init_acc_bias_);
  config_para.init_gyro_bias_ =
      Vec3d(ConfigParameters::Instance().imu_init_gyro_bias_,
            ConfigParameters::Instance().imu_init_gyro_bias_,
            ConfigParameters::Instance().imu_init_gyro_bias_);
  config_para.gravity_ = ConfigParameters::Instance().gravity_vector_;
  config_para.acc_noise_std_ =
      Vec3d(ConfigParameters::Instance().imu_acc_noise_std_,
            ConfigParameters::Instance().imu_acc_noise_std_,
            ConfigParameters::Instance().imu_acc_noise_std_);
  config_para.gyro_noise_std_ =
      Vec3d(ConfigParameters::Instance().imu_gyro_noise_std_,
            ConfigParameters::Instance().imu_gyro_noise_std_,
            ConfigParameters::Instance().imu_gyro_noise_std_);
  config_para.integration_noise_cov_ = Vec3d(1.0e-8, 1.0e-8, 1.0e-8);

  pre_integration_ptr_ = std::make_shared<PreIntegration>(config_para);
}

bool Localization::Init() {
  Mat4d init_pose;
  {
    std::lock_guard<std::mutex> lg(mutex_init_pose_optional_);
    if (!init_pose_optional_.has_value()) {
      LOG_EVERY_N(INFO, 10) << "Please set initialization pose for "
                               "localization by RViz -> 2D Pose Estimate";
      return false;
    }

    init_pose = init_pose_optional_.value();
    init_pose_optional_.reset();
  }

  const auto local_map = LoadLocalMap(init_pose);

  if (local_map->empty()) {
    return false;
  }

  UpdateLocalMapCloud(*local_map);

  matcher_->AddCloudToLocalMap({*local_map});
  const bool success = matcher_->Match(curr_cloud_cluster_ptr_, init_pose);

  const float fitness_score = matcher_->GetFitnessScore(2.0);

  if (success && fitness_score < 1.0) {
    curr_pose_ = init_pose;
    Mat15d cov = Mat15d::Zero();
    cov.block<3, 3>(0, 0) =
        Mat3d::Identity() * 1.0e-6 * 1.0e-6;  // rotation information matrix
    cov.block<3, 3>(3, 3) =
        Mat3d::Identity() * 1.0e-2 * 1.0e-2;  // velocity information matrix
    cov.block<3, 3>(6, 6) =
        Mat3d::Identity() * 1.0e-6 * 1.0e-6;  // position information matrix
    cov.block<3, 3>(9, 9) =
        Mat3d::Identity() *
        std::pow(0.1 * kDegree2Radian, 2);  // bias gyro information matrix
    cov.block<3, 3>(12, 12) =
        Mat3d::Identity() * 0.1 * 0.1;  // bias accel information matrix
    last_nav_state_.SetPose(init_pose);
    last_nav_state_.V_ = Vec3d::Zero();
    last_nav_state_.info_ = cov.inverse();
    last_nav_state_.timestamp_ = curr_time_us_;

    delta_pose_.setIdentity();
    last_pose_ = init_pose;
  } else {
    LOG(WARNING) << "Initial registration failed. "
                 << "Please try to continue initialization and give a better "
                    "initial pose.";
  }

  NavStateData curr_nav_state;
  curr_nav_state.timestamp_ = curr_time_us_;
  curr_nav_state.SetPose(init_pose);
  CacheResultToSystem(curr_nav_state);
  UpdateCurrentLidarCloud(*TransformPointCloud(
      VoxelGridCloud(curr_cloud_cluster_ptr_->ordered_cloud_, 0.5), init_pose));

  return success;
}

void Localization::Run() {
  LOG(INFO) << "\033[1;32m----> Localization Started.\033[0m";

  {
    std::lock_guard<std::mutex> lg(mutex_global_map_cloud_);
    std::string global_map_path = kDataPath + kGlobalMapFileName;
    global_map_cloud_ptr_ = LoadGlobalMap(global_map_path);

    if (global_map_cloud_ptr_) {
      constexpr auto filter_size = 0.3f;
      global_map_cloud_ptr_ =
          VoxelGridCloud(global_map_cloud_ptr_, filter_size);
    }
  }

  // If use tile map, load tile map indices file
  if (use_tile_map_) {
    LoadTileMapIndices();
  }

  while (rclcpp::ok()) {
    {
      std::unique_lock<std::mutex> lock(
          system_ptr_->mutex_cloud_cluster_deque_);
      while (system_ptr_->cloud_cluster_deque_.empty()) {
        if (!rclcpp::ok()) {
          return;
        }
        system_ptr_->cv_localization_.wait(lock);
      }

      curr_cloud_cluster_ptr_ = system_ptr_->cloud_cluster_deque_.front();
      system_ptr_->cloud_cluster_deque_.pop_front();
    }

    curr_time_us_ = curr_cloud_cluster_ptr_->timestamp_;

    if (!has_init_) {
      has_init_ = Init();

      // Initialization localization is successful. Initialize pre-integration.
      if (has_init_ && ConfigParameters::Instance().fusion_method_ ==
                           kFusionTightCouplingOptimization) {
        InitPreintegration();
      }
      continue;
    }

    Timer timer_load_map;
    auto local_map = LoadLocalMap(last_pose_);
    if (need_update_local_map_) {
      DLOG(INFO) << "Load local map use time(ms): " << timer_load_map.End();
      UpdateLocalMapCloud(*local_map);
      Timer timer_add_cloud;
      matcher_->AddCloudToLocalMap({*local_map});
      DLOG(INFO) << "Add local map to matcher use time(ms): "
                 << timer_add_cloud.End();
    }

    NavStateData predict_nav_state;

    if (ConfigParameters::Instance().fusion_method_ ==
        kFusionTightCouplingOptimization) {
      // use imu integrals as predicted values for registration
      predict_nav_state = IntegrateImuMeasures(last_nav_state_);
    } else if (ConfigParameters::Instance().fusion_method_ ==
               kFusionLooseCoupling) {
      // use imu integrated rotation
      predict_nav_state.SetPose(last_nav_state_.Pose() * delta_pose_);
      const Eigen::Quaterniond first_q =
          curr_cloud_cluster_ptr_->imu_data_.front().orientation_;
      const Eigen::Quaterniond end_q =
          curr_cloud_cluster_ptr_->imu_data_.back().orientation_;
      predict_nav_state.R_ = last_nav_state_.R_ * (first_q.inverse() * end_q);
    } else if (ConfigParameters::Instance().fusion_method_ ==
               kFusionTightCouplingKF) {
      LOG(FATAL) << "Kalman filter support will be coming soon!";
    } else {
      LOG(FATAL) << "Fusion method doesn't support: "
                 << ConfigParameters::Instance().fusion_method_;
    }

    // Performing pointcloud registration.
    // Use the predicted pose as the initial pose.
    Mat4d match_pose = predict_nav_state.Pose();
    if (!matcher_->Match(curr_cloud_cluster_ptr_, match_pose)) {
      continue;
    }

    NavStateData curr_nav_state;
    if (ConfigParameters::Instance().fusion_method_ ==
        kFusionTightCouplingOptimization) {
      curr_nav_state.SetPose(predict_nav_state.Pose());
      curr_nav_state.V_ = predict_nav_state.V_;
      curr_nav_state.timestamp_ = curr_time_us_;

      /// TODO: Dynamically adjust lidar information matrix

      // Performing pre-integration optimization
      Optimize(curr_nav_state, match_pose);
    } else if (ConfigParameters::Instance().fusion_method_ ==
               kFusionLooseCoupling) {
      curr_nav_state.SetPose(match_pose);
      curr_nav_state.V_ = predict_nav_state.V_;
      curr_nav_state.timestamp_ = curr_time_us_;
    } else {
      LOG(FATAL) << "Fusion method doesn't support: "
                 << ConfigParameters::Instance().fusion_method_;
    }

    // current lidar pose
    Mat4d curr_pose = curr_nav_state.Pose();
    delta_pose_ = last_pose_.inverse() * curr_pose;
    last_pose_ = curr_pose;

    // Cache result to System for visualization
    CacheResultToSystem(curr_nav_state);
    UpdateCurrentLidarCloud(*TransformPointCloud(
        VoxelGridCloud(curr_cloud_cluster_ptr_->ordered_cloud_, 0.5),
        curr_pose));

    last_nav_state_ = curr_nav_state;

    // reset pre-integration and reset bias
    if (ConfigParameters::Instance().fusion_method_ ==
        kFusionTightCouplingOptimization) {
      pre_integration_ptr_->Reset();
      pre_integration_ptr_->SetGyroAccBias(curr_nav_state.bg_,
                                           curr_nav_state.ba_);
    }
  }
}

PCLPointCloudXYZI::Ptr Localization::LoadGlobalMap(const std::string& path) {
  if (path.empty()) {
    return nullptr;
  }

  PCLPointCloudXYZI::Ptr global_cloud_ptr(new PCLPointCloudXYZI);
  const int res = pcl::io::loadPCDFile(path, *global_cloud_ptr);

  if (res == 0) {
    return global_cloud_ptr;
  }

  return nullptr;
}

PCLPointCloudXYZI::Ptr Localization::LoadLocalMap(const Mat4d& pose) {
  need_update_local_map_ = false;

  if (use_tile_map_) {
    const Vec2d position_xy = pose.block<2, 1>(0, 3);
    Vec2i curr_grid_index;
    curr_grid_index.x() = std::floor(
        (position_xy.x() - tile_map_grid_size_half_) / tile_map_grid_size_);
    curr_grid_index.y() = std::floor(
        (position_xy.y() - tile_map_grid_size_half_) / tile_map_grid_size_);

    // generate neighbor tile map indices
    const auto tile_map_indices = GenerateTileMapIndices(curr_grid_index, 1);

    // load new tile map cloud
    for (const Vec2i& tile_map_index : tile_map_indices) {
      if (map_data_indices_.find(tile_map_index) == map_data_indices_.end()) {
        continue;
      }

      if (local_map_data_.find(tile_map_index) == local_map_data_.end()) {
        auto tile_map_cloud = LoadTileMap(tile_map_index);

        if (!tile_map_cloud) {
          continue;
        }

        local_map_data_[tile_map_index] = VoxelGridCloud(
            tile_map_cloud, ConfigParameters::Instance()
                                .registration_local_map_cloud_filter_size_);
        need_update_local_map_ = true;
      }
    }

    // delete distant tile map cloud
    for (auto it = local_map_data_.begin(); it != local_map_data_.end();) {
      if ((it->first - curr_grid_index).cast<float>().norm() > 2) {
        local_map_data_.erase(it++);
        need_update_local_map_ = true;
      } else {
        ++it;
      }
    }

    static PCLPointCloudXYZI::Ptr map_cloud(new PCLPointCloudXYZI);

    // No need to update the map
    if (!need_update_local_map_) {
      return map_cloud;
    }

    map_cloud->clear();

    // Stitching tile map cloud
    for (const auto& data : local_map_data_) {
      if (!data.second) {
        continue;
      }
      *map_cloud += *data.second;
    }

    return map_cloud;
  } else {
    PCLPointCloudXYZI::Ptr map_cloud(new PCLPointCloudXYZI);

    static bool first = true;
    static pcl::CropBox<PCLPointXYZI> crop_box;

    if (first || !has_init_) {
      crop_box.setInputCloud(global_map_cloud_ptr_);
      local_map_edge_.clear();
      first = false;
    }

    if (local_map_edge_.empty()) {
      need_update_local_map_ = true;
    } else {
      for (int i = 0; i < 3; ++i) {
        if (std::fabs(pose(i, 3) - local_map_edge_[i]) > 50.0 &&
            std::fabs(pose(i, 3) - local_map_edge_[i + 3]) > 50.0) {
          continue;
        }
        need_update_local_map_ = true;
        break;
      }
    }

    if (!need_update_local_map_) {
      return nullptr;
    }

    local_map_edge_.resize(6);
    local_map_edge_[0] = pose(0, 3) - 100.0;
    local_map_edge_[1] = pose(1, 3) - 100.0;
    local_map_edge_[2] = pose(2, 3) - 100.0;

    local_map_edge_[3] = pose(0, 3) + 100.0;
    local_map_edge_[4] = pose(1, 3) + 100.0;
    local_map_edge_[5] = pose(2, 3) + 100.0;

    crop_box.setMin(
        Vec4d(local_map_edge_[0], local_map_edge_[1], local_map_edge_[2], 1.0)
            .cast<float>());
    crop_box.setMax(
        Vec4d(local_map_edge_[3], local_map_edge_[4], local_map_edge_[5], 1.0)
            .cast<float>());
    crop_box.filter(*map_cloud);

    return map_cloud;
  }
}

PCLPointCloudXYZI::Ptr Localization::LoadTileMap(const Vec2i& tile_map_index) {
  PCLPointCloudXYZI::Ptr cloud(new PCLPointCloudXYZI);

  std::string tile_map_path = kTileMapFolder + TILE_MAP_NAME(tile_map_index);
  int res = pcl::io::loadPCDFile(tile_map_path, *cloud);

  if (res == 0) {
    return cloud;
  }

  return nullptr;
}

std::vector<Vec2i> Localization::GenerateTileMapIndices(const Vec2i& index,
                                                        int step_size) {
  std::vector<Vec2i> neighbor_indices;
  for (int i = -step_size; i <= step_size; ++i) {
    for (int j = -step_size; j <= step_size; ++j) {
      Vec2i step(i, j);
      neighbor_indices.emplace_back(index + step);
    }
  }

  return neighbor_indices;
}

NavStateData Localization::IntegrateImuMeasures(
    const NavStateData& last_nav_state) const {
  pre_integration_ptr_->IntegrateDataSegment(
      curr_cloud_cluster_ptr_->imu_data_);
  return pre_integration_ptr_->Predict(last_nav_state);
}

void Localization::Optimize(NavStateData& curr_nav_state,
                            const Mat4d& lidar_pose) {
  using BlockSolverType = g2o::BlockSolverX;
  using LinearSolverType =
      g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;
  auto* solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);

  // === pre-integration error optimization ===
  // last rotation vertex
  auto vertex_rotation_last = new VertexRotation;
  vertex_rotation_last->setId(0);
  vertex_rotation_last->setEstimate(last_nav_state_.R_);
  optimizer.addVertex(vertex_rotation_last);

  // last velocity vertex
  auto vertex_velocity_last = new VertexVelocity;
  vertex_velocity_last->setId(1);
  vertex_velocity_last->setEstimate(last_nav_state_.V_);
  optimizer.addVertex(vertex_velocity_last);

  // last position vertex
  auto vertex_position_last = new VertexPosition;
  vertex_position_last->setId(2);
  vertex_position_last->setEstimate(last_nav_state_.P_);
  optimizer.addVertex(vertex_position_last);

  // current rotation vertex
  auto vertex_rotation_curr = new VertexRotation;
  vertex_rotation_curr->setId(3);
  vertex_rotation_curr->setEstimate(curr_nav_state.R_);
  optimizer.addVertex(vertex_rotation_curr);

  // current velocity vertex
  auto vertex_velocity_curr = new VertexVelocity;
  vertex_velocity_curr->setId(4);
  vertex_velocity_curr->setEstimate(curr_nav_state.V_);
  optimizer.addVertex(vertex_velocity_curr);

  // current position vertex
  auto vertex_position_curr = new VertexPosition;
  vertex_position_curr->setId(5);
  vertex_position_curr->setEstimate(curr_nav_state.P_);
  optimizer.addVertex(vertex_position_curr);

  // last gyro bias vertex
  auto vertex_bg_last = new VertexBiasGyro;
  vertex_bg_last->setId(6);
  vertex_bg_last->setEstimate(last_nav_state_.bg_);
  optimizer.addVertex(vertex_bg_last);

  // last accel bias vertex
  auto vertex_ba_last = new VertexBiasAccel;
  vertex_ba_last->setId(7);
  vertex_ba_last->setEstimate(last_nav_state_.ba_);
  optimizer.addVertex(vertex_ba_last);

  // current gyro bias vertex
  auto vertex_bg_curr = new VertexBiasGyro;
  vertex_bg_curr->setId(8);
  vertex_bg_curr->setEstimate(last_nav_state_.bg_);
  optimizer.addVertex(vertex_bg_curr);

  // current accel bias vertex
  auto vertex_ba_curr = new VertexBiasAccel;
  vertex_ba_curr->setId(9);
  vertex_ba_curr->setEstimate(last_nav_state_.ba_);
  optimizer.addVertex(vertex_ba_curr);

  // === prior last nav state edge ===
  auto* edge_prior_last_nav_state =
      new EdgePriorNavState(last_nav_state_, last_nav_state_.info_);
  edge_prior_last_nav_state->setVertex(0, vertex_rotation_last);
  edge_prior_last_nav_state->setVertex(1, vertex_velocity_last);
  edge_prior_last_nav_state->setVertex(2, vertex_position_last);
  edge_prior_last_nav_state->setVertex(3, vertex_bg_last);
  edge_prior_last_nav_state->setVertex(4, vertex_ba_last);
  optimizer.addEdge(edge_prior_last_nav_state);

  // === current lidar rotation edge ===
  auto* edge_lidar_rotation_edge = new EdgeRotation();
  edge_lidar_rotation_edge->setVertex(0, vertex_rotation_curr);
  edge_lidar_rotation_edge->setInformation(lidar_pose_info_.block<3, 3>(0, 0));
  edge_lidar_rotation_edge->setMeasurement(lidar_pose.block<3, 3>(0, 0));
  optimizer.addEdge(edge_lidar_rotation_edge);

  // === current lidar position edge ===
  auto* edge_lidar_position_edge = new EdgePosition();
  edge_lidar_position_edge->setVertex(0, vertex_position_curr);
  edge_lidar_position_edge->setInformation(lidar_pose_info_.block<3, 3>(3, 3));
  edge_lidar_position_edge->setMeasurement(lidar_pose.block<3, 1>(0, 3));
  optimizer.addEdge(edge_lidar_position_edge);

  // === pre-integration edge ===
  auto* edge_preintegration = new EdgePreIntegration(pre_integration_ptr_);
  edge_preintegration->setVertex(0, vertex_rotation_last);
  edge_preintegration->setVertex(1, vertex_velocity_last);
  edge_preintegration->setVertex(2, vertex_position_last);
  edge_preintegration->setVertex(3, vertex_bg_last);
  edge_preintegration->setVertex(4, vertex_ba_last);
  edge_preintegration->setVertex(5, vertex_rotation_curr);
  edge_preintegration->setVertex(6, vertex_velocity_curr);
  edge_preintegration->setVertex(7, vertex_position_curr);
  optimizer.addEdge(edge_preintegration);

  // === imu acc bias random walk edge ===
  Mat3d acc_bias_rw_info = Mat3d::Identity();
  acc_bias_rw_info =
      acc_bias_rw_info *
      (1.0 / std::pow(ConfigParameters::Instance().imu_acc_rw_noise_std_, 2.0));
  auto* edge_acc_bias_rw = new EdgeAccelBiasRW();
  edge_acc_bias_rw->setVertex(0, vertex_ba_last);
  edge_acc_bias_rw->setVertex(1, vertex_ba_curr);
  edge_acc_bias_rw->setInformation(acc_bias_rw_info);
  optimizer.addEdge(edge_acc_bias_rw);

  // === imu gyro bias random walk edge ===
  Mat3d gyro_bias_rw_info = Mat3d::Identity();
  gyro_bias_rw_info =
      gyro_bias_rw_info *
      (1.0 /
       std::pow(ConfigParameters::Instance().imu_gyro_rw_noise_std_, 2.0));
  auto* edge_gyro_bias_rw = new EdgeGyroBiasRW();
  edge_gyro_bias_rw->setVertex(0, vertex_bg_last);
  edge_gyro_bias_rw->setVertex(1, vertex_bg_curr);
  edge_gyro_bias_rw->setInformation(gyro_bias_rw_info);
  optimizer.addEdge(edge_gyro_bias_rw);

  // start to optimize
  optimizer.initializeOptimization();
  optimizer.optimize(ConfigParameters::Instance().frontend_fusion_opti_iters_);

  // get the results after optimization
  const Mat3d& curr_R_opti = vertex_rotation_curr->estimate();
  const Vec3d& curr_V_opti = vertex_velocity_curr->estimate();
  const Vec3d& curr_P_opti = vertex_position_curr->estimate();
  const Vec3d& curr_bg_opti = vertex_bg_curr->estimate();
  const Vec3d& curr_ba_opti = vertex_ba_curr->estimate();

  // Get all state posterior information
  // R_i, V_i, P_i, bg_i, ba_i, R_j, V_j, P_j, bg_j, ba_j
  //  0    3    6    9    12    15    18   21   24    27
  Eigen::Matrix<double, 30, 30> posterior_info =
      Eigen::Matrix<double, 30, 30>::Zero();

  // gyro bias random walk posterior info
  const Mat6d gyro_bias_posterior_info =
      edge_gyro_bias_rw->GetPosteriorInformation();
  posterior_info.block<3, 3>(9, 9) +=
      gyro_bias_posterior_info.block<3, 3>(0, 0);
  posterior_info.block<3, 3>(24, 24) +=
      gyro_bias_posterior_info.block<3, 3>(3, 3);
  posterior_info.block<3, 3>(9, 24) +=
      gyro_bias_posterior_info.block<3, 3>(0, 3);
  posterior_info.block<3, 3>(24, 9) +=
      gyro_bias_posterior_info.block<3, 3>(3, 0);

  // acc bias random walk posterior info
  const Mat6d acc_bias_posterior_info =
      edge_acc_bias_rw->GetPosteriorInformation();
  posterior_info.block<3, 3>(12, 12) +=
      acc_bias_posterior_info.block<3, 3>(0, 0);
  posterior_info.block<3, 3>(27, 27) +=
      acc_bias_posterior_info.block<3, 3>(3, 3);
  posterior_info.block<3, 3>(12, 27) +=
      acc_bias_posterior_info.block<3, 3>(0, 3);
  posterior_info.block<3, 3>(27, 12) +=
      acc_bias_posterior_info.block<3, 3>(3, 0);

  // pre-integration posterior info
  const Mat24d edge_preintegration_posterior_info =
      edge_preintegration->GetPosteriorInformation();
  posterior_info.block<24, 24>(0, 0) += edge_preintegration_posterior_info;

  // gps rotation posterior info
  const Mat3d edge_lidar_rotation_posterior_info =
      edge_lidar_rotation_edge->GetPosteriorInformation();
  posterior_info.block<3, 3>(15, 15) += edge_lidar_rotation_posterior_info;

  // gps position posterior info
  const Mat3d edge_lidar_position_posterior_info =
      edge_lidar_position_edge->GetPosteriorInformation();
  posterior_info.block<3, 3>(21, 21) += edge_lidar_position_posterior_info;

  // last nav state prior posterior info
  const Mat15d edge_last_nav_state_prior_posterior_info =
      edge_prior_last_nav_state->GetPosteriorInformation();
  posterior_info.block<15, 15>(0, 0) +=
      edge_last_nav_state_prior_posterior_info;

  curr_nav_state.R_ = curr_R_opti;
  curr_nav_state.V_ = curr_V_opti;
  curr_nav_state.P_ = curr_P_opti;
  curr_nav_state.bg_ = curr_bg_opti;
  curr_nav_state.ba_ = curr_ba_opti;
  curr_nav_state.info_ =
      Marginalize(posterior_info, 0, 14).block<15, 15>(15, 15);

  LOG_IF(WARNING, curr_ba_opti.x() > 1.0 || curr_ba_opti.y() > 1.0 ||
                      curr_ba_opti.z() > 1.0)
      << "The value of acc bias is too large, and the optimization may "
         "diverge.";
  LOG_IF(WARNING, curr_bg_opti.x() > 1.0 || curr_bg_opti.y() > 1.0 ||
                      curr_bg_opti.z() > 1.0)
      << "The value of gyro bias is too large, and the optimization may "
         "diverge.";
}

void Localization::CacheResultToSystem(const NavStateData& state) const {
  std::lock_guard<std::mutex> lg(
      system_ptr_->mutex_localization_results_deque_);
  system_ptr_->localization_results_deque_.emplace_back(
      std::make_shared<NavStateData>(state));
}

void Localization::UpdateLocalMapCloud(
    const PCLPointCloudXYZI& local_map_cloud) {
  std::lock_guard<std::mutex> lg(mutex_local_map_cloud_);
  local_map_cloud_ = local_map_cloud;
  need_update_local_map_visualization_ = true;
}

void Localization::UpdateCurrentLidarCloud(
    const PCLPointCloudXYZI& lidar_cloud) {
  std::lock_guard<std::mutex> lg(mutex_lidar_cloud_);
  lidar_cloud_ = lidar_cloud;
}

PCLPointCloudXYZI Localization::GetLocalCloudMap() {
  std::lock_guard<std::mutex> lg(mutex_local_map_cloud_);
  need_update_local_map_visualization_ = false;
  return local_map_cloud_;
}

bool Localization::NeedUpdateLocalMapVisualization() const {
  return need_update_local_map_visualization_;
}

PCLPointCloudXYZI Localization::GetGlobalCloudMap() {
  std::lock_guard<std::mutex> lg(mutex_global_map_cloud_);
  if (!global_map_cloud_ptr_) {
    return {};
  }

  return *global_map_cloud_ptr_;
}

PCLPointCloudXYZI Localization::GetCurrentLidarCloudMap() {
  std::lock_guard<std::mutex> lg(mutex_lidar_cloud_);
  return lidar_cloud_;
}

void Localization::LoadTileMapIndices() {
  const std::string file_name = kTileMapFolder + kTileMapIndicesFileName;

  std::ifstream file(file_name);

  CHECK(file.is_open()) << "Load tile map indices file failed!";

  while (!file.eof()) {
    int x, y;
    file >> x >> y;
    map_data_indices_.emplace(x, y);
  }

  file.close();
}
