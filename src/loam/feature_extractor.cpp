//
// Created by Zhang Zhimeng on 22-4-26.
//

#include "common/timer.h"
#include "common/constant_variable.h"
#include "loam/feature_extractor.h"

#include <glog/logging.h>

#include <execution>


namespace loam {

FeatureExtractor::FeatureExtractor(float corner_thr, float planar_thr,
                                   int lidar_horizontal_scan, int lidar_vertical_scan)
        : corner_threshold_(corner_thr), planar_threshold_(planar_thr),
          lidar_horizontal_scan_(lidar_horizontal_scan), lidar_vertical_scan_(lidar_vertical_scan) {
    CHECK_NE(corner_threshold_, FloatNaN);
    CHECK_NE(planar_threshold_, FloatNaN);
    CHECK_NE(lidar_horizontal_scan_, IntNaN);
    CHECK_NE(lidar_vertical_scan_, IntNaN);

    point_features_.resize(lidar_horizontal_scan_ * lidar_vertical_scan_);
    is_valid_points_ = new bool[lidar_horizontal_scan_ * lidar_vertical_scan_];
    is_corners_ = new bool[lidar_horizontal_scan_ * lidar_vertical_scan_];
}

FeatureExtractor::~FeatureExtractor() {
    delete[] is_valid_points_;
    delete[] is_corners_;
}

void FeatureExtractor::ExtractFeatures(PointcloudCluster &pointcloud_cluster) {
    memset(is_valid_points_, true, lidar_horizontal_scan_ * lidar_vertical_scan_);
    memset(is_corners_, false, lidar_horizontal_scan_ * lidar_vertical_scan_);

    Timer timer_compute_roughness;
    SelectValidPoints(pointcloud_cluster);
    ComputeRoughness(pointcloud_cluster);
    SelectFeatures(pointcloud_cluster);
    DLOG(INFO) << "FULL-LOAM compute roughness use time(ms)" << timer_compute_roughness.End();
}

void FeatureExtractor::ComputeRoughness(const PointcloudCluster &pointcloud_cluster) {
    const PCLPointCloudXYZI &ordered_pointcloud = pointcloud_cluster.ordered_cloud_;
    const auto &depth_vec = pointcloud_cluster.point_depth_vec_;

    const unsigned int num_points = ordered_pointcloud.points.size() - 5u;

    for (unsigned int i = 5u; i < num_points; ++i) {
        float roughness = depth_vec[i - 5] + depth_vec[i - 4] + depth_vec[i - 3] +
                          depth_vec[i - 2] + depth_vec[i - 1] + depth_vec[i + 1] +
                          depth_vec[i + 2] + depth_vec[i + 3] + depth_vec[i + 4] +
                          depth_vec[i + 5] - 10.0f * depth_vec[i];

        point_features_[i].roughness_ = roughness * roughness;
        point_features_[i].index_ = i;
    }
}


void FeatureExtractor::SelectValidPoints(const PointcloudCluster &pointcloud_cluster) {
    const unsigned int N = pointcloud_cluster.ordered_cloud_.size();

    is_valid_points_[0] = false;
    is_valid_points_[1] = false;
    is_valid_points_[2] = false;
    is_valid_points_[3] = false;
    is_valid_points_[4] = false;

    is_valid_points_[N - 1] = false;
    is_valid_points_[N - 2] = false;
    is_valid_points_[N - 3] = false;
    is_valid_points_[N - 4] = false;
    is_valid_points_[N - 5] = false;
    is_valid_points_[N - 6] = false;

    // 标记不稳定点，主要是遮挡点和平行点
    for (unsigned int i = 5; i < N - 6; ++i) {
        const float &depth_1 = pointcloud_cluster.point_depth_vec_[i];
        const float &depth_2 = pointcloud_cluster.point_depth_vec_[i + 1];
        const int col_diff = std::abs(pointcloud_cluster.point_col_index_vec_[i + 1]
                                      - pointcloud_cluster.point_col_index_vec_[i]
        );

        // 标记遮挡点
        if (col_diff < 10) {
            if (depth_1 - depth_2 > 0.3) {
                is_valid_points_[i] = false;
                is_valid_points_[i - 1] = false;
                is_valid_points_[i - 2] = false;
                is_valid_points_[i - 3] = false;
                is_valid_points_[i - 4] = false;
                is_valid_points_[i - 5] = false;
            } else if (depth_2 - depth_1 > 0.3) {
                is_valid_points_[i + 1] = false;
                is_valid_points_[i + 2] = false;
                is_valid_points_[i + 3] = false;
                is_valid_points_[i + 4] = false;
                is_valid_points_[i + 5] = false;
                is_valid_points_[i + 6] = false;
            }
        }

        // 标记平行点
        const float diff_1 = std::abs(pointcloud_cluster.point_depth_vec_[i - 1]
                                      - pointcloud_cluster.point_depth_vec_[i]);
        const float diff_2 = std::abs(pointcloud_cluster.point_depth_vec_[i + 1]
                                      - pointcloud_cluster.point_depth_vec_[i]);

        if (diff_1 > 0.02 * pointcloud_cluster.point_depth_vec_[i]
            && diff_2 > 0.02 * pointcloud_cluster.point_depth_vec_[i]) {
            is_valid_points_[i] = false;
        }
    }
}

void FeatureExtractor::SelectFeatures(PointcloudCluster &pointcloud_cluster) {
    pointcloud_cluster.planar_cloud_.clear();
    pointcloud_cluster.corner_cloud_.clear();

    for (int scan = 0; scan < lidar_vertical_scan_; ++scan) {
        PCLPointCloudXYZI::Ptr planar_cloud_scan(new PCLPointCloudXYZI);

        for (int i = 0; i < 6; ++i) {
            ///TODO: 关注一下int除法精度损失问题
            const int temp_index = (pointcloud_cluster.row_end_index_vec_[scan]
                                    - pointcloud_cluster.row_start_index_vec_[scan]) / 6;
            const int block_start_index = pointcloud_cluster.row_start_index_vec_[scan] + i * temp_index;
            const int block_end_index = pointcloud_cluster.row_start_index_vec_[scan] + (i + 1) * temp_index;

            if (block_start_index >= block_end_index) {
                continue;
            }

            std::sort(std::execution::par, point_features_.begin() + block_start_index,
                      point_features_.begin() + block_end_index,
                      [](const PointFeature &l, const PointFeature &r) -> bool {
                          return l.roughness_ < r.roughness_;
                      }
            );

            int number_large_corner = 0;
            for (int j = block_end_index; j >= block_start_index; --j) {
                const unsigned int index = point_features_[j].index_;

                if (point_features_[j].roughness_ > corner_threshold_ && is_valid_points_[index]) {
                    number_large_corner++;
                    if (number_large_corner <= 20) {
                        is_corners_[index] = true;
                        pointcloud_cluster.corner_cloud_.emplace_back(
                                pointcloud_cluster.ordered_cloud_.points[index]
                        );
                    } else {
                        break;
                    }

                    is_valid_points_[index] = false;
                    for (int k = 1; k <= 5; ++k) {
                        int col_diff = std::abs(pointcloud_cluster.point_col_index_vec_[index + k]
                                                - pointcloud_cluster.point_col_index_vec_[index + k - 1]);
                        if (col_diff > 10) {
                            break;
                        }

                        is_valid_points_[index + k] = false;
                    }

                    for (int k = -1; k >= -5; --k) {
                        int col_diff = std::abs(pointcloud_cluster.point_col_index_vec_[index + k]
                                                - pointcloud_cluster.point_col_index_vec_[index + k + 1]);

                        if (col_diff > 10) {
                            break;
                        }

                        is_valid_points_[index + k] = false;
                    }
                }
            }

            for (int j = block_start_index; j <= block_end_index; ++j) {
                const unsigned int index = point_features_[j].index_;

                if (is_valid_points_[index] && point_features_[j].roughness_ < planar_threshold_) {

                    is_valid_points_[index] = false;

                    for (int k = 1; k <= 5; ++k) {
                        int col_diff = std::abs(pointcloud_cluster.point_col_index_vec_[index + k]
                                                - pointcloud_cluster.point_col_index_vec_[index + k - 1]);

                        if (col_diff > 10) {
                            break;
                        }

                        is_valid_points_[index + k] = false;
                    }

                    for (int k = -1; k >= -5; --k) {
                        int col_diff = std::abs(pointcloud_cluster.point_col_index_vec_[index + k]
                                                - pointcloud_cluster.point_col_index_vec_[index + k + 1]);

                        if (col_diff > 10) {
                            break;
                        }

                        is_valid_points_[index + k] = false;
                    }
                }

                if (!is_corners_[index]) {
                    planar_cloud_scan->emplace_back(pointcloud_cluster.ordered_cloud_.points[index]);
                }
            }
        }

        pointcloud_cluster.planar_cloud_ += *planar_cloud_scan;
    }
}

}