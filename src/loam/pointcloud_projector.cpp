//
// Created by Zhang Zhimeng on 22-5-11.
//
#include "lidar/lidar_point_type.h"
#include "loam/pointcloud_projector.h"
#include "common/constant_variable.h"

#include <glog/logging.h>

#include <execution>
#include <utility>

namespace loam {

PointcloudProjector::PointcloudProjector(std::shared_ptr<LidarDistortionCorrector> lidar_distortion_corrector_ptr,
                                         int lidar_horizontal_scan, int lidar_vertical_scan,
                                         float lidar_horizontal_resolution,
                                         float min_distance, float max_distance)
        : lidar_distortion_corrector_ptr_(std::move(lidar_distortion_corrector_ptr)),
          lidar_vertical_scan_(lidar_vertical_scan), lidar_horizontal_scan_(lidar_horizontal_scan),
          lidar_horizontal_resolution_(lidar_horizontal_resolution), max_distance_(max_distance),
          min_distance_(min_distance) {
    CHECK_NOTNULL(lidar_distortion_corrector_ptr_);
    CHECK_NE(lidar_horizontal_scan_, IntNaN);
    CHECK_NE(lidar_vertical_scan_, IntNaN);
    CHECK_NE(lidar_horizontal_resolution_, FloatNaN);
    CHECK_NE(min_distance_, FloatNaN);
    CHECK_NE(max_distance_, FloatNaN);
    range_mat_.resize(lidar_vertical_scan_, lidar_horizontal_scan_);
}

void PointcloudProjector::Project(PointcloudCluster &pointcloud_cluster) {
    pointcloud_cluster.ordered_cloud_.clear();
    range_mat_.setConstant(std::numeric_limits<float>::max());

    const unsigned int temp_size = lidar_vertical_scan_ * lidar_horizontal_scan_;
    if (pointcloud_cluster.point_depth_vec_.size() != temp_size) {
        pointcloud_cluster.point_depth_vec_.resize(lidar_vertical_scan_ * lidar_horizontal_scan_);
    }

    if (pointcloud_cluster.point_col_index_vec_.size() != temp_size) {
        pointcloud_cluster.point_col_index_vec_.resize(lidar_vertical_scan_ * lidar_horizontal_scan_, 0);
    }

    if (pointcloud_cluster.row_end_index_vec_.size() != static_cast<size_t>(lidar_vertical_scan_)) {
        pointcloud_cluster.row_end_index_vec_.resize(lidar_vertical_scan_);
    }

    if (pointcloud_cluster.row_start_index_vec_.size() != static_cast<size_t>(lidar_vertical_scan_)) {
        pointcloud_cluster.row_start_index_vec_.resize(lidar_vertical_scan_);
    }

    temp_pointcloud_.points.clear();
    temp_pointcloud_.points.resize(lidar_vertical_scan_ * lidar_horizontal_scan_);

    std::for_each(std::execution::seq, pointcloud_cluster.raw_cloud_.begin(), pointcloud_cluster.raw_cloud_.end(),
                  [&](const auto &point) {
                      const float x = point.x;
                      const float y = point.y;
                      const float z = point.z;

                      float depth = std::sqrt(x * x + y * y + z * z);

                      if (depth < min_distance_ || depth > max_distance_) {
                          return;
                      }

                      int row = point.ring;
                      int col = static_cast<int>(std::round(FastAtan2(y, x) / lidar_horizontal_resolution_)) +
                                lidar_horizontal_scan_ / 2;

                      if (col >= lidar_horizontal_scan_) {
                          col -= lidar_horizontal_scan_;
                      }

                      // Debug: Compare our row calculation method with lidar ring
//        std::cout << "x, y, z: " << pointcloud_cluster.raw_cloud_->points[i].x
//                  << " " << pointcloud_cluster.raw_cloud_->points[i].y
//                  << " " << pointcloud_cluster.raw_cloud_->points[i].z << std::endl;
//
//        std::cout << "row: " << row << "    " << "ring: " <<
//                  static_cast<unsigned int>(pointcloud_cluster.raw_cloud_->points[i].ring) << std::endl
//                  << std::endl;

                      if (row >= lidar_vertical_scan_ || row < 0 || col < 0 || col >= lidar_horizontal_scan_)
                          return;

                      int index = row * lidar_horizontal_scan_ + col;

                      if (range_mat_(row, col) != std::numeric_limits<float>::max())
                          return;

                      float pt_x, pt_y, pt_z;
                      pt_x = point.x;
                      pt_y = point.y;
                      pt_z = point.z;

                      float relative_time = point.time;

                      if (!lidar_distortion_corrector_ptr_->ProcessPoint(pt_x, pt_y, pt_z, pt_x, pt_y, pt_z,
                                                                         relative_time)) {
                          return;
                      }

                      range_mat_(row, col) = depth;
                      temp_pointcloud_.points[index].x = pt_x;
                      temp_pointcloud_.points[index].y = pt_y;
                      temp_pointcloud_.points[index].z = pt_z;
                      temp_pointcloud_.points[index].intensity = point.intensity;
                  }
    );

    int count = 0;
    for (int row = 0; row < lidar_vertical_scan_; ++row) {
        pointcloud_cluster.row_start_index_vec_[row] = count + 5;

        for (int col = 0; col < lidar_horizontal_scan_; ++col) {
            if (range_mat_(row, col) == std::numeric_limits<float>::max()) {
                continue;
            }

            pointcloud_cluster.point_depth_vec_[count] = range_mat_(row, col);
            pointcloud_cluster.ordered_cloud_.emplace_back(
                    temp_pointcloud_[row * lidar_horizontal_scan_ + col]
            );
            pointcloud_cluster.point_col_index_vec_[count] = col;

            ++count;
        }

        pointcloud_cluster.row_end_index_vec_[row] = count - 6;
    }
}

}