//
// Created by Zhang Zhimeng on 23-11-2.
//

#ifndef FUNNY_LIDAR_SLAM_PREPROCESSING_H
#define FUNNY_LIDAR_SLAM_PREPROCESSING_H

#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "lidar/lidar_distortion_corrector.h"
#include "lidar/lidar_model.h"
#include "lidar/lidar_point_type.h"
#include "loam/feature_extractor.h"
#include "loam/pointcloud_projector.h"
#include "slam/system.h"

class PreProcessing {
 public:
  using DataSearcherIMU = DataSearcher<IMUData>;

 public:
  PreProcessing() = delete;

  explicit PreProcessing(System* system_ptr);

  ~PreProcessing() = default;

  void Run();

 private:
  void InitVoxelGridFilter();

  static pcl::PointCloud<PointXYZIRT>::Ptr ConvertMessageToCloud(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_ros_ptr);

  /*!
   * Compute lidar every point offset time
   * note: Only supports clockwise rotation lidar scan
   * @param cloud
   * @param lidar_rate
   */
  static void ComputePointOffsetTime(
      const pcl::PointCloud<PointXYZIRT>::Ptr& cloud, double lidar_rate = 10.0);

  static std::pair<float, float> GetLidarPointMinMaxOffsetTime(
      const pcl::PointCloud<PointXYZIRT>::Ptr& cloud);

 private:
  // voxel grid filter
  pcl::VoxelGrid<PCLPointXYZI> corner_voxel_filter_;
  pcl::VoxelGrid<PCLPointXYZI> planer_voxel_filter_;

  PCLPointCloudXYZIRT::Ptr temp_cloud_ptr_ = nullptr;

  std::shared_ptr<loam::FeatureExtractor> feature_extractor_ptr_ = nullptr;
  std::shared_ptr<loam::PointcloudProjector> cloud_projector_ptr_ = nullptr;
  std::shared_ptr<LidarDistortionCorrector> lidar_distortion_corrector_ptr_ =
      nullptr;

  System* system_ptr_ = nullptr;
};

#endif  // FUNNY_LIDAR_SLAM_PREPROCESSING_H
