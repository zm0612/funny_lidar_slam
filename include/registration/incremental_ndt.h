//
// Created by Zhang Zhimeng on 24-5-15.
//

#ifndef FUNNY_LIDAR_SLAM_INCREMENTAL_NDT_H
#define FUNNY_LIDAR_SLAM_INCREMENTAL_NDT_H

#include <utility>

#include "registration/registration_interface.h"
#include "common/pointcloud_utility.h"
#include "common/compare_function.h"
#include "common/hash_function.h"
#include "common/math_function.h"

class IncrementalNDT final : public RegistrationInterface {
public:
    using PointType = PCLPointXYZI;
    using CloudType = PCLPointCloudXYZI;

public:
    IncrementalNDT(double voxel_size, double res_outlier_threshold, float source_cloud_filter_size,
                   double rotation_converge_thres, double position_converge_thres,
                   int min_points_in_voxel, int max_points_in_voxel,
                   int min_effective_pts, int capacity, int max_iteration,
                   bool is_localization_mode = false) {
        CHECK_NE(voxel_size, DoubleNaN);
        CHECK_NE(res_outlier_threshold, DoubleNaN);
        CHECK_NE(rotation_converge_thres, DoubleNaN);
        CHECK_NE(position_converge_thres, DoubleNaN);
        CHECK_NE(source_cloud_filter_size, FloatNaN);
        CHECK_NE(min_points_in_voxel, IntNaN);
        CHECK_NE(max_points_in_voxel, IntNaN);
        CHECK_NE(min_effective_pts, IntNaN);
        CHECK_NE(max_iteration, IntNaN);
        CHECK_NE(capacity, IntNaN);
        voxel_size_ = voxel_size;
        inv_voxel_size_ = 1.0 / voxel_size_;
        is_localization_mode_ = is_localization_mode;
        res_outlier_threshold_ = res_outlier_threshold;
        rotation_converge_thres_ = rotation_converge_thres;
        position_converge_thres_ = position_converge_thres;
        source_cloud_filter_size_ = source_cloud_filter_size;
        min_points_in_voxel_ = min_points_in_voxel;
        max_points_in_voxel_ = max_points_in_voxel;
        min_effective_pts_ = min_effective_pts;
        max_iteration_ = max_iteration;
        capacity_ = static_cast<size_t>(capacity);
        GenerateNearbyGrids();

        DLOG(INFO) << "voxel_size: " << voxel_size_;
        DLOG(INFO) << "is_localization_mode: " << is_localization_mode_;
        DLOG(INFO) << "res_outlier_threshold: " << res_outlier_threshold_;
        DLOG(INFO) << "rotation_converge_thres: " << rotation_converge_thres_;
        DLOG(INFO) << "position_converge_thres: " << position_converge_thres_;
        DLOG(INFO) << "source_cloud_filter_size: " << source_cloud_filter_size_;
        DLOG(INFO) << "min_points_in_voxel: " << min_points_in_voxel_;
        DLOG(INFO) << "max_points_in_voxel: " << max_points_in_voxel_;
        DLOG(INFO) << "min_effective_pts: " << min_effective_pts_;
        DLOG(INFO) << "max_iteration: " << max_iteration_;
        DLOG(INFO) << "capacity: " << capacity_;
    }

private:
    struct VoxelData {
        explicit VoxelData(const Vec3d& pt) {
            points_.emplace_back(pt);
            num_points_ = 1;
        }

        void AddPoint(const Vec3d& pt) {
            points_.emplace_back(pt);
            if (!ndt_estimated_) {
                num_points_++;
            }
        }

        std::vector<Vec3d> points_; // 内部点，多于一定数量之后再估计均值和协方差
        Vec3d mu_ = Vec3d::Zero(); // 均值
        Mat3d sigma_ = Mat3d::Zero(); // 协方差
        Mat3d information_ = Mat3d::Zero(); // 协方差之逆

        bool ndt_estimated_ = false; // NDT是否已经估计
        int num_points_ = 0; // 总共的点数，用于更新估计
    };

    using KeyType = Vec3i; // 体素的索引
    using KeyAndData = std::pair<KeyType, VoxelData>; // 预定义

private:
    template <typename Getter>
    void ComputeMeanAndCov(const std::vector<Vec3d>& data, Vec3d& mean,
                           Mat3d& cov, Getter&& getter) const {
        using D = Vec3d;
        using E = Mat3d;
        size_t len = data.size();
        CHECK_GT(len, 1u);

        mean = std::accumulate(data.begin(), data.end(), Vec3d::Zero().eval(),
                               [&getter](const D& sum, const auto& data) -> D {
                                   return sum + getter(data);
                               }
        ) / static_cast<double>(len);
        cov = std::accumulate(data.begin(), data.end(), E::Zero().eval(),
                              [&mean, &getter](const E& sum, const auto& data) -> E {
                                  D v = getter(data) - mean;
                                  return sum + v * v.transpose();
                              }
        ) / static_cast<double>(len - 1);
    }

    static void UpdateMeanAndCov(int hist_m, int curr_n, const Vec3d& hist_mean, const Mat3d& hist_var,
                                 const Vec3d& curr_mean, const Mat3d& curr_var, Vec3d& new_mean, Mat3d& new_var) {
        CHECK_GT(hist_m, 0);
        CHECK_GT(curr_n, 0);
        new_mean = (hist_m * hist_mean + curr_n * curr_mean) / (hist_m + curr_n);
        new_var = (hist_m * (hist_var + (hist_mean - new_mean) * (hist_mean - new_mean).transpose()) +
                curr_n * (curr_var + (curr_mean - new_mean) * (curr_mean - new_mean).transpose())) /
            (hist_m + curr_n);
    }

    void GenerateNearbyGrids() {
        nearby_grids_step_index_ = {
            KeyType(0, 0, 0), KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
            KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)
        };
    }

    /// 更新体素内部数据, 根据新加入的pts和历史的估计情况来确定自己的估计
    void UpdateVoxel(VoxelData& v) const {
        if (flag_first_scan_) {
            if (v.points_.size() > 1u) {
                ComputeMeanAndCov(v.points_, v.mu_, v.sigma_, [](const Vec3d& p) { return p; });
                v.information_ = (v.sigma_ + Mat3d::Identity() * 1.0e-3).inverse();
            } else {
                v.mu_ = v.points_[0];
                v.information_ = Mat3d::Identity() * 1.0e2;
            }

            v.ndt_estimated_ = true;
            v.points_.clear();
            return;
        }

        if (v.ndt_estimated_ && v.num_points_ > max_points_in_voxel_) {
            return;
        }

        if (!v.ndt_estimated_ && static_cast<int>(v.points_.size()) > min_points_in_voxel_) {
            ComputeMeanAndCov(v.points_, v.mu_, v.sigma_, [](const Vec3d& p) { return p; });
            v.information_ = (v.sigma_ + Mat3d::Identity() * 1e-3).inverse(); // 避免出nan
            v.ndt_estimated_ = true;
            v.points_.clear();
        } else if (v.ndt_estimated_ && static_cast<int>(v.points_.size()) > min_points_in_voxel_) {
            Vec3d cur_mu, new_mu;
            Mat3d cur_var, new_var;
            ComputeMeanAndCov(v.points_, cur_mu, cur_var, [](const Vec3d& p) { return p; });
            UpdateMeanAndCov(v.num_points_, static_cast<int>(v.points_.size()),
                             v.mu_, v.sigma_, cur_mu, cur_var, new_mu, new_var);

            v.mu_ = new_mu;
            v.sigma_ = new_var;
            v.num_points_ += static_cast<int>(v.points_.size());
            v.points_.clear();

            Eigen::JacobiSVD svd(v.sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Vec3d lambda = svd.singularValues();
            if (lambda[1] < lambda[0] * 1e-3) {
                lambda[1] = lambda[0] * 1e-3;
            }

            if (lambda[2] < lambda[0] * 1e-3) {
                lambda[2] = lambda[0] * 1e-3;
            }

            Mat3d inv_lambda = Vec3d(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();
            v.information_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
        }
    }

public:
    void AddCloudToLocalMap(const std::initializer_list<PCLPointCloudXYZI>& cloud_list) override {
        CHECK_EQ(cloud_list.size(), 1) << "IncrementalNDT only need undistorted cloud in map frame";

        auto cloud_world_full = *cloud_list.begin();
        auto cloud_world = VoxelGridCloud(cloud_world_full, source_cloud_filter_size_);

        if (is_localization_mode_) {
            kdtree_flann_.setInputCloud(cloud_world);
        }

        std::set<KeyType, LessVec3i> active_voxels; // 记录哪些voxel被更新
        for (const auto& point : cloud_world->points) {
            Vec3d point_eigen = point.getVector3fMap().cast<double>();
            Vec3i key = (point_eigen * inv_voxel_size_).cast<int>();
            auto iter = grids_.find(key);
            if (iter == grids_.end()) {
                // voxel not exists, add points, update cache
                data_.emplace_front(key, VoxelData(point_eigen));
                grids_.insert({key, data_.begin()});

                if (data_.size() >= capacity_) {
                    grids_.erase(data_.back().first);
                    data_.pop_back();
                }
            } else {
                // voxel exists, add points, update cache
                iter->second->second.AddPoint(point_eigen);
                data_.splice(data_.begin(), data_, iter->second); // 更新的那个放到最前
                iter->second = data_.begin(); // grids时也指向最前
            }

            active_voxels.emplace(key);
        }

        std::for_each(std::execution::par_unseq, active_voxels.begin(), active_voxels.end(),
                      [this](const auto& key) {
                          UpdateVoxel(grids_[key]->second);
                      }
        );

        if (is_localization_mode_) {
            flag_first_scan_ = true;
        } else {
            flag_first_scan_ = false;
        }
    }

    bool Match(const PointcloudClusterPtr& source_cloud_cluster, Mat4d& T) override {
        CHECK(!grids_.empty());

        source_cloud = VoxelGridCloud(source_cloud_cluster->ordered_cloud_.makeShared(),
                                      source_cloud_filter_size_);

        Mat4d pose = T;

        static constexpr size_t num_residual_per_point = 7; // search nearby 7 voxel

        std::vector<int> index(source_cloud->points.size());
        std::iota(index.begin(), index.end(), 0);

        const size_t total_size = index.size() * num_residual_per_point;

        bool has_converge = false;

        for (int iter = 0; iter < max_iteration_; ++iter) {
            std::vector<Vec3d> error_vec(total_size);
            std::vector<Mat3d> information_vec(total_size);
            std::vector<bool> effect_pts(total_size, false);
            std::vector<Eigen::Matrix<double, 3, 6>> jacobian_vec(total_size);

            std::for_each(std::execution::par_unseq, index.begin(), index.end(),
                          [&](size_t idx) {
                              const Vec3d point = source_cloud->points[idx].getVector3fMap().cast<double>();
                              const Vec3d point_transformed = pose.block<3, 3>(0, 0) * point + pose.block<3, 1>(0, 3);
                              const Vec3i key = (point_transformed * inv_voxel_size_).cast<int>();

                              for (size_t i = 0u; i < num_residual_per_point; ++i) {
                                  const Vec3i real_key = key + nearby_grids_step_index_[i];
                                  auto grid_iter = grids_.find(real_key);
                                  const size_t real_idx = idx * num_residual_per_point + i;

                                  if (grid_iter != grids_.end() && grid_iter->second->second.ndt_estimated_) {
                                      const auto& voxel = grid_iter->second->second;
                                      const Vec3d error = point_transformed - voxel.mu_;

                                      const double residual = error.transpose() * voxel.information_ * error;
                                      if (std::isnan(residual) || residual > res_outlier_threshold_) {
                                          effect_pts[real_idx] = false;
                                          continue;
                                      }

                                      Eigen::Matrix<double, 3, 6> jacobian;
                                      jacobian.block<3, 3>(0, 0) = -pose.block<3, 3>(0, 0) * SO3Hat(point);
                                      jacobian.block<3, 3>(0, 3) = Mat3d::Identity();

                                      jacobian_vec[real_idx] = jacobian;
                                      error_vec[real_idx] = error;
                                      information_vec[real_idx] = voxel.information_;
                                      effect_pts[real_idx] = true;
                                  } else {
                                      effect_pts[real_idx] = false;
                                  }
                              }
                          }
            );

            double total_res = 0.0;
            int effective_num = 0;

            Mat6d H = Mat6d::Zero();
            Vec6d err = Vec6d::Zero();

            for (size_t idx = 0; idx < effect_pts.size(); ++idx) {
                if (!effect_pts[idx]) {
                    continue;
                }

                total_res += error_vec[idx].transpose() * information_vec[idx] * error_vec[idx];
                effective_num++;

                H += jacobian_vec[idx].transpose() * information_vec[idx] * jacobian_vec[idx];
                err += -jacobian_vec[idx].transpose() * information_vec[idx] * error_vec[idx];
            }

            if (effective_num < min_effective_pts_) {
                T = pose;
                return false;
            }

            const Vec6d dx = H.inverse() * err;
            pose.block<3, 3>(0, 0) *= SO3Exp(dx.head(3));
            pose.block<3, 1>(0, 3) += dx.tail(3);

            if (dx.head(3).norm() < rotation_converge_thres_ && dx.tail(3).norm() < position_converge_thres_) {
                DLOG(INFO) << "num iter= " << std::setw(2) << iter
                    << " total res= " << std::setprecision(5) << total_res
                    << " mean res= " << std::setprecision(5) << total_res / static_cast<double>(effective_num)
                    << " valid point= " << effective_num;
                has_converge = true;
                break;
            }
        }

        has_converge = true;

        if (has_converge && !is_localization_mode_) {
            const auto source_cloud_transformed = TransformPointCloud(*source_cloud, T);
            AddCloudToLocalMap({source_cloud_transformed});
        } else if (!has_converge) {
            LOG(WARNING) << "Incremental NDT match failed!\n";
        }

        T = pose;
        final_transformation_ = T;
        return has_converge;
    }

    /*!
     * @brief Get fitness score
     * @param max_range Nearest neighbor search threshold
     *
     * @note Only be used in localization mode, otherwise the output is infinite
     */
    [[nodiscard]] float GetFitnessScore(float max_range) const override {
        if (!is_localization_mode_) {
            return FloatNaN;
        }

        float fitness_score = 0.0f;

        const CloudType::Ptr transformed_cloud_ptr(new CloudType);
        *transformed_cloud_ptr = TransformPointCloud(*source_cloud, final_transformation_);

        std::vector<int> nn_indices(1);
        std::vector<float> nn_dists(1);

        int nr = 0;

        for (unsigned int i = 0; i < transformed_cloud_ptr->size(); ++i) {
            kdtree_flann_.nearestKSearch(transformed_cloud_ptr->points[i], 1, nn_indices, nn_dists);

            if (nn_dists.front() <= max_range) {
                fitness_score += nn_dists.front();
                nr++;
            }
        }

        if (nr > 0)
            return fitness_score / static_cast<float>(nr);
        else
            return (std::numeric_limits<float>::max());
    }

private:
    std::list<KeyAndData> data_{};
    std::vector<KeyType> nearby_grids_step_index_{};
    std::unordered_map<KeyType, std::list<KeyAndData>::iterator, SpatialHashFunction> grids_{};

    int max_iteration_{}; // 最大迭代次数
    int min_effective_pts_{}; // 最近邻点数阈值
    int min_points_in_voxel_{}; // 每个栅格中最小点数
    int max_points_in_voxel_{}; // 每个栅格中最大点数
    double voxel_size_{}; // 体素大小
    double inv_voxel_size_{}; // 体素大小之逆
    double res_outlier_threshold_{}; // 异常值拒绝阈值
    double rotation_converge_thres_{};
    double position_converge_thres_{};
    float source_cloud_filter_size_{};
    size_t capacity_{}; // 缓存的体素数量

    bool flag_first_scan_ = true; // 首帧点云特殊处理

    pcl::KdTreeFLANN<PointType> kdtree_flann_{};
    Eigen::Matrix<double, 4, 4> final_transformation_{};
    CloudType::Ptr source_cloud = nullptr;
    bool is_localization_mode_ = false;
};

#endif //FUNNY_LIDAR_SLAM_INCREMENTAL_NDT_H
