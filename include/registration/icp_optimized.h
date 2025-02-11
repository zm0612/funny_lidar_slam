//
// Created by Zhang Zhimeng on 24-4-3.
//

#ifndef FUNNY_LIDAR_SLAM_ICP_OPTIMIZED_H
#define FUNNY_LIDAR_SLAM_ICP_OPTIMIZED_H

#include <pcl/kdtree/kdtree_flann.h>

#include <deque>
#include <iomanip>

#include "common/constant_variable.h"
#include "common/math_function.h"
#include "common/pointcloud_utility.h"
#include "registration/registration_interface.h"

template <typename Type = float>
class IcpOptimized final : public RegistrationInterface {
 public:
  using PointType = PCLPointXYZI;
  using CloudType = PCLPointCloudXYZI;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  IcpOptimized(unsigned int max_iterations, unsigned int local_map_size,
               float map_cloud_filter_size, float source_cloud_filter_size,
               Type max_correspond_distance, Type position_converge_thres,
               Type rotation_converge_thres, Type rot_thre_add_cloud,
               Type dist_thre_add_cloud, bool is_localization_mode = false)
      : max_iterations_(max_iterations),
        local_map_size_(local_map_size),
        map_cloud_filter_size_(map_cloud_filter_size),
        source_cloud_filter_size_(source_cloud_filter_size),
        max_correspond_distance_(max_correspond_distance),
        position_converge_thres_(position_converge_thres),
        rotation_converge_thres_(rotation_converge_thres),
        rot_thre_add_cloud_(rot_thre_add_cloud),
        dist_thre_add_cloud_(dist_thre_add_cloud),
        local_map_ptr_(new CloudType),
        source_cloud_ptr_(new CloudType),
        is_localization_mode_(is_localization_mode) {
    CHECK_NE(max_iterations_, IntNaN);
    CHECK_NE(local_map_size_, IntNaN);
    CHECK_NE(map_cloud_filter_size_, FloatNaN);
    CHECK_NE(source_cloud_filter_size_, FloatNaN);
    CHECK_NE(max_correspond_distance_, std::numeric_limits<Type>::max());
    CHECK_NE(position_converge_thres_, std::numeric_limits<Type>::max());
    CHECK_NE(rotation_converge_thres_, std::numeric_limits<Type>::max());
    CHECK_NE(rot_thre_add_cloud_, std::numeric_limits<Type>::max());
    CHECK_NE(dist_thre_add_cloud_, std::numeric_limits<Type>::max());
    DLOG(INFO) << "max iteration: " << max_iterations_;
    DLOG(INFO) << "local_map_size: " << local_map_size_;
    DLOG(INFO) << "map_cloud_filter_size: " << map_cloud_filter_size_;
    DLOG(INFO) << "source_cloud_filter_size: " << source_cloud_filter_size_;
    DLOG(INFO) << "max_correspond_distance: " << max_correspond_distance_;
    DLOG(INFO) << "position_converge_thres: " << position_converge_thres_;
    DLOG(INFO) << "rotation_converge_thres: " << rotation_converge_thres_;
    DLOG(INFO) << "rot_thre_add_cloud: " << rot_thre_add_cloud_;
    DLOG(INFO) << "dist_thre_add_cloud: " << dist_thre_add_cloud_;
    DLOG(INFO) << "is_localization_mode: " << is_localization_mode_;
  }

  bool Match(const PointcloudClusterPtr& source_cloud_cluster,
             Mat4d& T) override {
    CHECK_GT(source_cloud_cluster->ordered_cloud_.size(), 10u);
    has_converge_ = false;
    source_cloud_ptr_ =
        VoxelGridCloud(source_cloud_cluster->ordered_cloud_.makeShared(),
                       source_cloud_filter_size_);

    CloudType::Ptr transformed_cloud_ptr(new CloudType);
    Eigen::Matrix<Type, 4, 4> T_temp = T.cast<Type>();

    for (unsigned int iteration = 0; iteration < max_iterations_; ++iteration) {
      transformed_cloud_ptr = TransformPointCloud(source_cloud_ptr_, T_temp);

      Eigen::Matrix<Type, 6, 6> Hessian = Eigen::Matrix<Type, 6, 6>::Zero();
      Eigen::Matrix<Type, 6, 1> B = Eigen::Matrix<Type, 6, 1>::Zero();

      const unsigned int cloud_size = transformed_cloud_ptr->size();
      std::vector<Eigen::Matrix<Type, 6, 6>> H_all(
          cloud_size, Eigen::Matrix<Type, 6, 6>::Zero());
      std::vector<Eigen::Matrix<Type, 6, 1>> B_all(
          cloud_size, Eigen::Matrix<Type, 6, 1>::Zero());
      std::vector<Eigen::Matrix<Type, 3, 1>> error_vec(
          cloud_size, Eigen::Matrix<Type, 3, 1>::Zero());
      std::vector<bool> effect_pts(cloud_size, false);

      std::vector<size_t> indices(cloud_size);
      std::iota(indices.begin(), indices.end(), 0);

      std::for_each(
          std::execution::par, indices.begin(), indices.end(),
          [&](const size_t& index) {
            const PointType& origin_point = source_cloud_ptr_->points[index];

            const PointType& transformed_point =
                transformed_cloud_ptr->at(index);
            std::vector<float> resultant_distances;
            std::vector<int> indices;
            kdtree_flann_.nearestKSearch(transformed_point, 1, indices,
                                         resultant_distances);

            if (resultant_distances.front() > max_correspond_distance_) {
              return;
            }

            Eigen::Matrix<Type, 3, 1> nearest_point =
                local_map_ptr_->at(indices.front())
                    .getVector3fMap()
                    .cast<Type>();
            Eigen::Matrix<Type, 3, 1> point_eigen =
                transformed_point.getVector3fMap().cast<Type>();
            Eigen::Matrix<Type, 3, 1> origin_point_eigen =
                origin_point.getVector3fMap().cast<Type>();
            Eigen::Matrix<Type, 3, 1> error = point_eigen - nearest_point;

            Eigen::Matrix<Type, 3, 6> Jacobian =
                Eigen::Matrix<Type, 3, 6>::Zero();

            Jacobian.leftCols(3) = Eigen::Matrix<Type, 3, 3>::Identity();
            Jacobian.rightCols(3) =
                -T_temp.template block<3, 3>(0, 0) * SO3Hat(origin_point_eigen);

            Eigen::Matrix<Type, 6, 6> H = Jacobian.transpose() * Jacobian;
            Eigen::Matrix<Type, 6, 1> B = -Jacobian.transpose() * error;

            H_all[index] = H;
            B_all[index] = B;
            effect_pts[index] = true;
            error_vec[index] = error;
          });

      CHECK_EQ(H_all.size(), B_all.size());

      double total_res = 0.0;
      int effective_num = 0;

      for (unsigned int i = 0; i < cloud_size; ++i) {
        if (!effect_pts[i]) {
          continue;
        }

        Hessian += H_all[i];
        B += B_all[i];

        effective_num++;
        total_res += error_vec[i].norm();
      }

      if (Hessian.determinant() == 0) {
        continue;
      }

      Eigen::Matrix<Type, 6, 1> delta_x = Hessian.inverse() * B;

      T_temp.template block<3, 1>(0, 3) =
          T_temp.template block<3, 1>(0, 3) + delta_x.head(3);
      T_temp.template block<3, 3>(0, 0) *= SO3Exp(delta_x.tail(3)).matrix();

      if (delta_x.tail(3).norm() < rotation_converge_thres_ &&
          delta_x.head(3).norm() < position_converge_thres_) {
        DLOG(INFO) << "num iter= " << std::setw(2) << iteration
                   << " total point size= " << std::setprecision(5)
                   << cloud_size << " total res= " << std::setprecision(5)
                   << total_res << " mean res= " << std::setprecision(5)
                   << total_res / static_cast<double>(effective_num)
                   << " valid point= " << effective_num;

        has_converge_ = true;
        break;
      }
    }

    final_transformation_ = T_temp;
    T = T_temp.template cast<double>();

    if (has_converge_ && IsNeedAddCloud(final_transformation_) &&
        !is_localization_mode_) {
      CloudType::Ptr transform_cloud(new CloudType);
      transform_cloud = TransformPointCloud(
          source_cloud_ptr_, final_transformation_.template cast<double>());
      AddCloudToLocalMap({*transform_cloud});
    } else if (!has_converge_) {
      LOG(WARNING) << "ICP optimized match failed!\n";
    }

    return has_converge_;
  }

  void AddCloudToLocalMap(
      const std::initializer_list<PCLPointCloudXYZI>& cloud_list) override {
    CHECK_EQ(cloud_list.size(), 1)
        << "IcpOptimized only need undistorted cloud";

    const auto new_cloud = (cloud_list.begin())->makeShared();

    if (is_localization_mode_) {
      *local_map_ptr_ = *new_cloud;
    } else {
      cloud_deque_.push_back(new_cloud);

      if (cloud_deque_.size() > local_map_size_) {
        cloud_deque_.pop_front();
      }

      local_map_ptr_->clear();

      for (const auto& it : cloud_deque_) {
        const auto cloud_downsample =
            VoxelGridCloud(it, map_cloud_filter_size_);
        *local_map_ptr_ += *it;
      }
    }

    local_map_ptr_ = VoxelGridCloud(local_map_ptr_, map_cloud_filter_size_);
    kdtree_flann_.setInputCloud(local_map_ptr_);
  }

  [[nodiscard]] float GetFitnessScore(float max_range) const override {
    float fitness_score = 0.0f;

    CloudType::Ptr transformed_cloud_ptr(new CloudType);
    *transformed_cloud_ptr = TransformPointCloud(
        *source_cloud_ptr_, final_transformation_.template cast<double>());

    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);

    int nr = 0;

    for (unsigned int i = 0; i < transformed_cloud_ptr->size(); ++i) {
      kdtree_flann_.nearestKSearch(transformed_cloud_ptr->points[i], 1,
                                   nn_indices, nn_dists);

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
  bool IsNeedAddCloud(const Eigen::Matrix<Type, 4, 4>& T) {
    static Eigen::Matrix<Type, 4, 4> last_T = T;

    const Eigen::Matrix<Type, 3, 3> R_delta =
        last_T.template block<3, 3>(0, 0).inverse() *
        T.template block<3, 3>(0, 0);
    const Eigen::Matrix<Type, 3, 1> euler_delta = RotationMatrixToRPY(R_delta);

    if ((T.template block<3, 1>(0, 3) - last_T.template block<3, 1>(0, 3))
                .norm() > Type(dist_thre_add_cloud_) ||
        std::fabs(euler_delta.x()) > Type(rot_thre_add_cloud_) ||
        std::fabs(euler_delta.y()) > Type(rot_thre_add_cloud_) ||
        std::fabs(euler_delta.z()) > Type(rot_thre_add_cloud_)) {
      last_T = T;
      return true;
    } else {
      return false;
    }
  }

 private:
  const unsigned int max_iterations_{30}, local_map_size_{40};
  const float map_cloud_filter_size_{0.5}, source_cloud_filter_size_{0.4};
  const Type max_correspond_distance_{2.0};
  const Type position_converge_thres_{0.01};
  const Type rotation_converge_thres_{0.05};
  const Type rot_thre_add_cloud_{0.2}, dist_thre_add_cloud_{1.0};
  bool has_converge_{false};

  CloudType::Ptr local_map_ptr_ = nullptr;
  CloudType::Ptr source_cloud_ptr_ = nullptr;

  std::deque<CloudType::Ptr> cloud_deque_;

  Eigen::Matrix<Type, 4, 4> final_transformation_{};
  pcl::KdTreeFLANN<PointType> kdtree_flann_{};
  bool is_localization_mode_ = false;
};

#endif  // FUNNY_LIDAR_SLAM_ICP_OPTIMIZED_H
