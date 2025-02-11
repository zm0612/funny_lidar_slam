//
// Created by Zhang Zhimeng on 23-11-2.
//

#include "slam/preprocessing.h"

#include <pcl/common/point_tests.h>  // for pcl::isFinite
#include <pcl_conversions/pcl_conversions.h>

#include "common/constant_variable.h"
#include "common/pointcloud_utility.h"
#include "common/timer.h"
#include "imu/imu_interpolator.h"
#include "lidar/pointcloud_cluster.h"
#include "slam/config_parameters.h"

PreProcessing::PreProcessing(System* system_ptr) : system_ptr_(system_ptr) {
  lidar_distortion_corrector_ptr_ = std::make_shared<LidarDistortionCorrector>(
      ConfigParameters::Instance().calibration_lidar_to_imu_);

  if (ConfigParameters::Instance().registration_and_searcher_mode_ ==
      kLoamFull_KdTree) {
    cloud_projector_ptr_ = std::make_shared<loam::PointcloudProjector>(
        lidar_distortion_corrector_ptr_,
        LidarModel::Instance()->horizon_scan_num_,
        LidarModel::Instance()->vertical_scan_num_,
        LidarModel::Instance()->h_res_,
        ConfigParameters::Instance().lidar_use_min_dist_,
        ConfigParameters::Instance().lidar_use_max_dist_);

    feature_extractor_ptr_ = std::make_shared<loam::FeatureExtractor>(
        ConfigParameters::Instance().loam_feature_corner_thres_,
        ConfigParameters::Instance().loam_feature_planar_thres_,
        LidarModel::Instance()->horizon_scan_num_,
        LidarModel::Instance()->vertical_scan_num_);
  }

  InitVoxelGridFilter();
}

void PreProcessing::InitVoxelGridFilter() {
  auto corner_size =
      ConfigParameters::Instance().loam_feature_corner_voxel_filter_size_;
  auto planer_size =
      ConfigParameters::Instance().loam_feature_planar_voxel_filter_size_;

  corner_voxel_filter_.setLeafSize(corner_size, corner_size, corner_size);
  planer_voxel_filter_.setLeafSize(planer_size, planer_size, planer_size);
}

void PreProcessing::Run() {
  LOG(INFO) << "\033[1;32m----> PreProcessing Started.\033[0m";

  while (rclcpp::ok()) {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr raw_cloud_ros;

    static bool imu_data_blocked = false;
    static TimeStampUs last_cloud_timestamp =
        std::numeric_limits<TimeStampUs>::max();

    {
      // wait new data
      std::unique_lock<std::mutex> lock(system_ptr_->mutex_raw_cloud_deque_);
      while (system_ptr_->raw_cloud_deque_.empty() || imu_data_blocked) {
        if (!rclcpp::ok()) {
          return;
        }
        imu_data_blocked = false;
        system_ptr_->cv_preprocessing_.wait(lock);
      }

      raw_cloud_ros = system_ptr_->raw_cloud_deque_.front();
    }

    if (!raw_cloud_ros) {
      continue;
    }

    // Convert lidar ros message to unified format
    PCLPointCloudXYZIRT::Ptr raw_cloud_ptr;

    if (!temp_cloud_ptr_) {
      raw_cloud_ptr = ConvertMessageToCloud(raw_cloud_ros);
    } else {
      raw_cloud_ptr = temp_cloud_ptr_;
    }

    const auto min_max_offset_time =
        GetLidarPointMinMaxOffsetTime(raw_cloud_ptr);

    TimeStampUs curr_cloud_timestamp = raw_cloud_ptr->header.stamp;

    auto cloud_start_timestamp = static_cast<uint64_t>(
        static_cast<int64_t>(raw_cloud_ptr->header.stamp) +
        static_cast<int64_t>(min_max_offset_time.first * 1.0e6));
    auto cloud_end_timestamp = static_cast<uint64_t>(
        static_cast<int64_t>(raw_cloud_ptr->header.stamp) +
        static_cast<int64_t>(min_max_offset_time.second * 1.0e6));

    // In order for lidar distortion correct, get imu data completely.
    if (curr_cloud_timestamp < cloud_start_timestamp) {
      cloud_start_timestamp = curr_cloud_timestamp;
    } else if (curr_cloud_timestamp > cloud_end_timestamp) {
      cloud_end_timestamp = curr_cloud_timestamp;
    }

    DLOG(INFO) << std::setprecision(15) << "last   cloud  timestamp: "
               << static_cast<double>(last_cloud_timestamp) *
                      kMicroseconds2Seconds;
    DLOG(INFO) << std::setprecision(15) << "current cloud timestamp: "
               << static_cast<double>(curr_cloud_timestamp) *
                      kMicroseconds2Seconds;
    DLOG(INFO) << std::setprecision(15) << "  cloud start timestamp: "
               << static_cast<double>(cloud_start_timestamp) *
                      kMicroseconds2Seconds;
    DLOG(INFO) << std::setprecision(15) << "  cloud  end  timestamp: "
               << static_cast<double>(cloud_end_timestamp) *
                      kMicroseconds2Seconds;

    // Make sure the pointcloud is not empty
    if (raw_cloud_ptr->empty()) {
      std::unique_lock<std::mutex> lock(system_ptr_->mutex_raw_cloud_deque_);
      system_ptr_->raw_cloud_deque_.pop_front();
      temp_cloud_ptr_ = nullptr;
      LOG(WARNING) << "Lidar cloud data empty";
      continue;
    }

    // Make sure the lidar data is later the oldest imu data
    const auto oldest_imu_data =
        system_ptr_->imu_data_searcher_ptr_->OldestData();
    if (!oldest_imu_data.has_value() ||
        oldest_imu_data.value().timestamp_ > cloud_start_timestamp) {
      std::unique_lock<std::mutex> lock(system_ptr_->mutex_raw_cloud_deque_);
      system_ptr_->raw_cloud_deque_.pop_front();
      temp_cloud_ptr_ = nullptr;
      // When the system lags and there is no time to process lidar data,
      // the following warning may be displayed.
      LOG(WARNING) << "Lidar data queue may be blocked";
      continue;
    }

    // Make sure the lidar data is earlier the latest imu data
    const auto latest_imu_data =
        system_ptr_->imu_data_searcher_ptr_->LatestData();
    if (!latest_imu_data.has_value() ||
        latest_imu_data.value().timestamp_ < cloud_end_timestamp) {
      temp_cloud_ptr_ = raw_cloud_ptr;
      imu_data_blocked = true;
      continue;
    }

    // Pop out the data to be processed
    {
      std::unique_lock<std::mutex> lock(system_ptr_->mutex_raw_cloud_deque_);
      system_ptr_->raw_cloud_deque_.pop_front();
      temp_cloud_ptr_ = nullptr;
    }

    const auto imu_data_segment =
        system_ptr_->imu_data_searcher_ptr_->GetDataSegment(
            cloud_start_timestamp, cloud_end_timestamp);

    auto imu_data_searcher =
        std::make_shared<DataSearcherIMU>(imu_data_segment);

    // Set the reference time point for de-distortion
    lidar_distortion_corrector_ptr_->SetDataSearcher(imu_data_searcher);
    lidar_distortion_corrector_ptr_->SetRefTime(curr_cloud_timestamp);

    // Creat new point cloud cluster
    auto cloud_cluster_ptr = std::make_shared<PointcloudCluster>();
    cloud_cluster_ptr->raw_cloud_ = *raw_cloud_ptr;
    cloud_cluster_ptr->timestamp_ = curr_cloud_timestamp;

    if (last_cloud_timestamp == std::numeric_limits<TimeStampUs>::max()) {
      IMUData data_l, data_r, curr_data;
      system_ptr_->imu_data_searcher_ptr_->SearchNearestTwoData(
          curr_cloud_timestamp, data_l, data_r);
      curr_data = IMUInterpolator(data_l, data_r, curr_cloud_timestamp);
      cloud_cluster_ptr->imu_data_.emplace_back(std::move(curr_data));
    } else {
      CHECK_GT(curr_cloud_timestamp, last_cloud_timestamp);
      cloud_cluster_ptr->imu_data_ =
          system_ptr_->imu_data_searcher_ptr_->GetDataSegment(
              last_cloud_timestamp, curr_cloud_timestamp);
    }

    DLOG(INFO) << "cloud cluster imu data size: "
               << cloud_cluster_ptr->imu_data_.size();

    last_cloud_timestamp = curr_cloud_timestamp;

    // When there is no need to extract features, enter the following judgment
    if (ConfigParameters::Instance().registration_and_searcher_mode_ ==
            kPointToPlane_IVOX ||
        ConfigParameters::Instance().registration_and_searcher_mode_ ==
            kPointToPlane_KdTree ||
        ConfigParameters::Instance().registration_and_searcher_mode_ ==
            kIcpOptimized ||
        ConfigParameters::Instance().registration_and_searcher_mode_ ==
            kIncrementalNDT) {
      // reserve memory
      cloud_cluster_ptr->planar_cloud_.reserve(
          cloud_cluster_ptr->raw_cloud_.size());
      cloud_cluster_ptr->ordered_cloud_.reserve(
          cloud_cluster_ptr->raw_cloud_.size());

      for (unsigned int i = 0; i < cloud_cluster_ptr->raw_cloud_.size(); ++i) {
        float pt_x, pt_y, pt_z, pt_i;
        pt_x = cloud_cluster_ptr->raw_cloud_[i].x;
        pt_y = cloud_cluster_ptr->raw_cloud_[i].y;
        pt_z = cloud_cluster_ptr->raw_cloud_[i].z;
        pt_i = cloud_cluster_ptr->raw_cloud_[i].intensity;

        float depth = std::sqrt(pt_x * pt_x + pt_y * pt_y + pt_z * pt_z);
        if (depth < ConfigParameters::Instance().lidar_use_min_dist_ ||
            depth > ConfigParameters::Instance().lidar_use_max_dist_) {
          continue;
        }

        // undistorted this point
        float relative_time = cloud_cluster_ptr->raw_cloud_[i].time;
        if (!lidar_distortion_corrector_ptr_->ProcessPoint(
                pt_x, pt_y, pt_z, pt_x, pt_y, pt_z, relative_time)) {
          continue;
        }

        PCLPointXYZI point_xyzi;
        point_xyzi.x = pt_x;
        point_xyzi.y = pt_y;
        point_xyzi.z = pt_z;
        point_xyzi.intensity = pt_i;

        // skip some points
        if (i % ConfigParameters::Instance().lidar_point_jump_span_ == 0) {
          cloud_cluster_ptr->planar_cloud_.emplace_back(point_xyzi);
        }

        cloud_cluster_ptr->ordered_cloud_.emplace_back(point_xyzi);
      }

      planer_voxel_filter_.setInputCloud(
          cloud_cluster_ptr->planar_cloud_.makeShared());
      planer_voxel_filter_.filter(cloud_cluster_ptr->planar_cloud_);
    } else if (ConfigParameters::Instance().registration_and_searcher_mode_ ==
               kLoamFull_KdTree) {
      // The following are used to extract loam features
      // Order pointcloud
      cloud_projector_ptr_->Project(*cloud_cluster_ptr);

      // Extract load features
      feature_extractor_ptr_->ExtractFeatures(*cloud_cluster_ptr);

      corner_voxel_filter_.setInputCloud(
          cloud_cluster_ptr->corner_cloud_.makeShared());
      corner_voxel_filter_.filter(cloud_cluster_ptr->corner_cloud_);
      planer_voxel_filter_.setInputCloud(
          cloud_cluster_ptr->planar_cloud_.makeShared());
      planer_voxel_filter_.filter(cloud_cluster_ptr->planar_cloud_);
    } else {
      LOG(FATAL) << "Unsupported type! "
                    "You can choose one of the following: "
                    "PointToPlane_IVOX, LoamFull_KdTree, IcpOptimized, "
                    "kIncrementalNDT";
    }

    {
      // Cache data and remind the frontend thread to process it
      std::lock_guard<std::mutex> lg(system_ptr_->mutex_cloud_cluster_deque_);
      system_ptr_->cloud_cluster_deque_.emplace_back(
          std::move(cloud_cluster_ptr));

      if (system_ptr_->slam_mode_ == SLAM_MODE::MAPPING) {
        system_ptr_->cv_frontend_.notify_one();
      } else if (system_ptr_->slam_mode_ == SLAM_MODE::LOCALIZATION) {
        system_ptr_->cv_localization_.notify_one();
      } else {
        LOG(FATAL) << "SLAM Mode Error";
      }
    }

    lidar_distortion_corrector_ptr_->SetDataSearcher(nullptr);
  }
}

pcl::PointCloud<PointXYZIRT>::Ptr PreProcessing::ConvertMessageToCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_ros_ptr) {
  if (LidarModel::Instance()->lidar_sensor_type_ ==
      LidarModel::LidarSensorType::VELODYNE) {
    pcl::PointCloud<VelodynePointXYZIRT> cloud_velodyne;
    pcl::fromROSMsg(*cloud_ros_ptr, cloud_velodyne);

    if (!cloud_velodyne.is_dense) {
      RemoveNaNFromPointCloud(cloud_velodyne, cloud_velodyne);
    }

    const size_t cloud_size = cloud_velodyne.size();
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_in_ptr(
        new pcl::PointCloud<PointXYZIRT>);
    cloud_in_ptr->resize(cloud_size);

    std::vector<size_t> indices(cloud_size);
    std::iota(indices.begin(), indices.end(), 0);

    std::for_each(std::execution::par, indices.begin(), indices.end(),
                  [&](const size_t& index) {
                    PointXYZIRT point_xyzirt{};
                    point_xyzirt.x = cloud_velodyne[index].x;
                    point_xyzirt.y = cloud_velodyne[index].y;
                    point_xyzirt.z = cloud_velodyne[index].z;
                    point_xyzirt.intensity = cloud_velodyne[index].intensity;
                    point_xyzirt.ring =
                        static_cast<uint8_t>(cloud_velodyne[index].ring);
                    point_xyzirt.time = static_cast<float>(
                        cloud_velodyne[index].time *
                        ConfigParameters::Instance().lidar_point_time_scale_);
                    cloud_in_ptr->at(index) = point_xyzirt;
                  });

    cloud_in_ptr->header = cloud_velodyne.header;
    cloud_in_ptr->is_dense = true;

    if (cloud_in_ptr->points.back().time <= 0.0f) {
      LOG(WARNING) << "Lidar cloud last point offset time <= 0.0";
      ComputePointOffsetTime(cloud_in_ptr, 10.0);
    }

    return cloud_in_ptr;
  } else if (LidarModel::Instance()->lidar_sensor_type_ ==
             LidarModel::LidarSensorType::OUSTER) {
    pcl::PointCloud<OusterPointXYZIRT> cloud_ouster;
    pcl::fromROSMsg(*cloud_ros_ptr, cloud_ouster);

    if (!cloud_ouster.is_dense) {
      RemoveNaNFromPointCloud(cloud_ouster, cloud_ouster);
    }

    unsigned int cloud_size = cloud_ouster.size();
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_in_ptr(
        new pcl::PointCloud<PointXYZIRT>);
    cloud_in_ptr->resize(cloud_size);

    std::vector<size_t> indices(cloud_size);
    std::iota(indices.begin(), indices.end(), 0);

    std::for_each(std::execution::par, indices.begin(), indices.end(),
                  [&](const size_t& index) {
                    PointXYZIRT point_xyzirt{};
                    point_xyzirt.x = cloud_ouster[index].x;
                    point_xyzirt.y = cloud_ouster[index].y;
                    point_xyzirt.z = cloud_ouster[index].z;
                    point_xyzirt.intensity = cloud_ouster[index].intensity;
                    point_xyzirt.ring =
                        static_cast<uint8_t>(cloud_ouster[index].ring);
                    point_xyzirt.time = static_cast<float>(
                        cloud_ouster[index].t *
                        ConfigParameters::Instance().lidar_point_time_scale_);
                    cloud_in_ptr->at(index) = point_xyzirt;
                  });

    cloud_in_ptr->header = cloud_ouster.header;
    cloud_in_ptr->is_dense = true;

    return cloud_in_ptr;
  } else if (LidarModel::Instance()->lidar_sensor_type_ ==
             LidarModel::LidarSensorType::LeiShen) {
    pcl::PointCloud<LsPointXYZIRT> cloud_ls;
    pcl::fromROSMsg(*cloud_ros_ptr, cloud_ls);

    if (!cloud_ls.is_dense) {
      RemoveNaNFromPointCloud(cloud_ls, cloud_ls);
    }

    unsigned int cloud_size = cloud_ls.size();
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_in_ptr(
        new pcl::PointCloud<PointXYZIRT>);
    cloud_in_ptr->resize(cloud_size);

    std::vector<size_t> indices(cloud_size);
    std::iota(indices.begin(), indices.end(), 0);

    std::for_each(std::execution::par, indices.begin(), indices.end(),
                  [&](const size_t& index) {
                    PointXYZIRT point_xyzirt{};
                    point_xyzirt.x = cloud_ls[index].x;
                    point_xyzirt.y = cloud_ls[index].y;
                    point_xyzirt.z = cloud_ls[index].z;
                    point_xyzirt.intensity = cloud_ls[index].intensity;
                    point_xyzirt.ring =
                        static_cast<uint8_t>(cloud_ls[index].ring);
                    point_xyzirt.time = static_cast<float>(
                        cloud_ls[index].timestamp *
                        ConfigParameters::Instance().lidar_point_time_scale_);
                    cloud_in_ptr->at(index) = point_xyzirt;
                  });

    cloud_in_ptr->header = cloud_ls.header;
    cloud_in_ptr->is_dense = true;

    return cloud_in_ptr;
  } else if (LidarModel::Instance()->lidar_sensor_type_ ==
             LidarModel::LidarSensorType::RoboSense) {
    /// Notation: robosense lidar frame timestamp is scan last point time!
    ///           And the point timestamp is second from UNIX!

    pcl::PointCloud<RsPointXYZIRT> cloud_rs;
    pcl::fromROSMsg(*cloud_ros_ptr, cloud_rs);

    if (!cloud_rs.is_dense) {
      RemoveNaNFromPointCloud(cloud_rs, cloud_rs);
    }

    cloud_rs.header.stamp =
        static_cast<std::uint64_t>(cloud_rs[0].timestamp * 1.0e6);

    unsigned int cloud_size = cloud_rs.size();
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_in_ptr(
        new pcl::PointCloud<PointXYZIRT>);
    cloud_in_ptr->resize(cloud_size);

    std::vector<size_t> indices(cloud_size);
    std::iota(indices.begin(), indices.end(), 0);

    std::for_each(std::execution::par, indices.begin(), indices.end(),
                  [&](const size_t& index) {
                    PointXYZIRT point_xyzirt{};
                    point_xyzirt.x = cloud_rs[index].x;
                    point_xyzirt.y = cloud_rs[index].y;
                    point_xyzirt.z = cloud_rs[index].z;
                    point_xyzirt.intensity = cloud_rs[index].intensity;
                    point_xyzirt.ring =
                        static_cast<uint8_t>(cloud_rs[index].ring);
                    point_xyzirt.time = static_cast<float>(
                        (cloud_rs[index].timestamp - cloud_rs[0].timestamp) *
                        ConfigParameters::Instance().lidar_point_time_scale_);
                    cloud_in_ptr->at(index) = point_xyzirt;
                  });

    cloud_in_ptr->header = cloud_rs.header;
    cloud_in_ptr->is_dense = true;

    return cloud_in_ptr;
  } else if (LidarModel::Instance()->lidar_sensor_type_ ==
             LidarModel::LidarSensorType::LIVOX_MID_360) {
    pcl::PointCloud<LivoxMid360PointXYZITLT> cloud_livox_mid_360;
    pcl::fromROSMsg(*cloud_ros_ptr, cloud_livox_mid_360);

    if (!cloud_livox_mid_360.is_dense) {
      RemoveNaNFromPointCloud(cloud_livox_mid_360, cloud_livox_mid_360);
    }

    unsigned int cloud_size = cloud_livox_mid_360.size();
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_in_ptr(
        new pcl::PointCloud<PointXYZIRT>);
    cloud_in_ptr->resize(cloud_size);

    std::vector<size_t> indices(cloud_size);
    std::iota(indices.begin(), indices.end(), 0);

    std::for_each(std::execution::par, indices.begin(), indices.end(),
                  [&](const size_t& index) {
                    PointXYZIRT point_xyzirt{};
                    point_xyzirt.x = cloud_livox_mid_360[index].x;
                    point_xyzirt.y = cloud_livox_mid_360[index].y;
                    point_xyzirt.z = cloud_livox_mid_360[index].z;
                    point_xyzirt.intensity =
                        cloud_livox_mid_360[index].intensity;
                    point_xyzirt.ring = 0;
                    point_xyzirt.time = static_cast<float>(
                        (cloud_livox_mid_360[index].timestamp -
                         cloud_livox_mid_360[0].timestamp) *
                        ConfigParameters::Instance().lidar_point_time_scale_);
                    cloud_in_ptr->at(index) = point_xyzirt;
                  });

    cloud_in_ptr->header = cloud_livox_mid_360.header;
    cloud_in_ptr->is_dense = true;

    return cloud_in_ptr;
  } else if (LidarModel::Instance()->lidar_sensor_type_ ==
             LidarModel::LidarSensorType::LIVOX_AVIA) {
    pcl::PointCloud<LivoxPointXYZITLT> cloud_livox;
    pcl::fromROSMsg(*cloud_ros_ptr, cloud_livox);

    const size_t point_size = cloud_livox.size();

    pcl::PointCloud<PointXYZIRT>::Ptr cloud_in_ptr(
        new pcl::PointCloud<PointXYZIRT>());
    cloud_in_ptr->reserve(point_size);

    /// TODO: config
    uint8_t num_scans_ = 6;

    for (size_t i = 0; i < point_size; ++i) {
      if ((cloud_livox.points[i].line < num_scans_) &&
          ((cloud_livox.points[i].tag & 0x30) == 0x10 ||
           (cloud_livox.points[i].tag & 0x30) == 0x00)) {
        PointXYZIRT point_xyzirt{};
        point_xyzirt.x = cloud_livox.points[i].x;
        point_xyzirt.y = cloud_livox.points[i].y;
        point_xyzirt.z = cloud_livox.points[i].z;
        point_xyzirt.intensity = cloud_livox.points[i].intensity;
        point_xyzirt.time = static_cast<float>(
            static_cast<double>(cloud_livox.points[i].time) *
            ConfigParameters::Instance().lidar_point_time_scale_);
        cloud_in_ptr->emplace_back(point_xyzirt);
      }
    }

    cloud_in_ptr->header = cloud_livox.header;
    cloud_in_ptr->is_dense = true;

    return cloud_in_ptr;
  } else if (LidarModel::Instance()->lidar_sensor_type_ ==
             LidarModel::LidarSensorType::None) {
    pcl::PointCloud<pcl::PointXYZI> cloud_none;
    pcl::fromROSMsg(*cloud_ros_ptr, cloud_none);

    size_t point_size = cloud_none.size();

    pcl::PointCloud<PointXYZIRT>::Ptr cloud_in_ptr(
        new pcl::PointCloud<PointXYZIRT>);
    cloud_in_ptr->reserve(point_size);
    for (size_t i = 0; i < point_size; ++i) {
      if (!pcl::isFinite(cloud_none[i])) {
        continue;
      }

      PointXYZIRT point_xyzirt{};
      point_xyzirt.x = cloud_none[i].x;
      point_xyzirt.y = cloud_none[i].y;
      point_xyzirt.z = cloud_none[i].z;
      point_xyzirt.intensity = cloud_none[i].intensity;
      point_xyzirt.time = 0.0;

      float xy = std::sqrt(cloud_none[i].x * cloud_none[i].x +
                           cloud_none[i].y * cloud_none[i].y);
      const int row =
          static_cast<int>(std::round((FastAtan2(cloud_none[i].z, xy) +
                                       LidarModel::Instance()->lower_angle_) /
                                      LidarModel::Instance()->v_res_));

      if (row >= LidarModel::Instance()->vertical_scan_num_ || row < 0)
        continue;

      point_xyzirt.ring = static_cast<uint8_t>(row);

      cloud_in_ptr->emplace_back(point_xyzirt);
    }

    cloud_in_ptr->header = cloud_none.header;
    cloud_in_ptr->is_dense = true;

    if (cloud_in_ptr->points.back().time <= 0.0f) {
      ComputePointOffsetTime(cloud_in_ptr, 10.0);
    }

    return cloud_in_ptr;
  } else {
    LOG(FATAL) << "Not support lidar type";
  }
  return nullptr;
}

void PreProcessing::ComputePointOffsetTime(
    const pcl::PointCloud<PointXYZIRT>::Ptr& cloud, const double lidar_rate) {
  CHECK_NE(cloud, nullptr);

  const int lidar_scan_num = LidarModel::Instance()->vertical_scan_num_;
  const auto cloud_size = cloud->size();
  const double lidar_omega = 2.0 * M_PI * lidar_rate;  // lidar angular velocity
  std::vector<bool> is_first(lidar_scan_num, true);
  std::vector<double> yaw_first_scan(lidar_scan_num,
                                     0.0);  // yaw of first scan point
  std::vector<float> time_last(lidar_scan_num, 0.0f);  // last offset time

  for (size_t i = 0; i < cloud_size; i++) {
    const int ring = cloud->points[i].ring;

    if (ring >= lidar_scan_num) {
      LOG(WARNING) << "lidar cloud point ring invalid";
      continue;
    }

    const double yaw = std::atan2(cloud->points[i].y, cloud->points[i].x);

    if (is_first[ring]) {
      yaw_first_scan[ring] = yaw;
      is_first[ring] = false;
      time_last[ring] = 0.0f;
      continue;
    }

    if (yaw <= yaw_first_scan[ring]) {
      cloud->points[i].time =
          static_cast<float>((yaw_first_scan[ring] - yaw) / lidar_omega);
    } else {
      cloud->points[i].time = static_cast<float>(
          (yaw_first_scan[ring] - yaw + 2.0 * M_PI) / lidar_omega);
    }

    if (cloud->points[i].time < time_last[ring]) {
      cloud->points[i].time += static_cast<float>(2.0 * M_PI / lidar_omega);
    }

    time_last[ring] = cloud->points[i].time;
  }
}

std::pair<float, float> PreProcessing::GetLidarPointMinMaxOffsetTime(
    const pcl::PointCloud<PointXYZIRT>::Ptr& cloud) {
  CHECK_NE(cloud->empty(), true);
  float min = cloud->points[0].time;
  float max = cloud->points[0].time;

  for (const auto point : cloud->points) {
    if (point.time < min) {
      min = point.time;
    }

    if (point.time > max) {
      max = point.time;
    }
  }

  return {min, max};
}
