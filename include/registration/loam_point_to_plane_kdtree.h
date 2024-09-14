//
// Created by Zhang Zhimeng on 23-12-14.
//

#ifndef FUNNY_LIDAR_SLAM_LOAM_POINT_TO_PLANE_KDTREE_H
#define FUNNY_LIDAR_SLAM_LOAM_POINT_TO_PLANE_KDTREE_H

#include "common/timer.h"
#include "common/data_type.h"
#include "common/constant_variable.h"
#include "common/pointcloud_utility.h"
#include "registration/registration_interface.h"

#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <glog/logging.h>

#include <iomanip>
#include <iostream>
#include <algorithm>
#include <execution>

template <typename Type=float>
class LoamPointToPlaneKdtree final : public RegistrationInterface {
public:
    using PointType = PCLPointXYZI;
    using CloudType = PCLPointCloudXYZI;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit LoamPointToPlaneKdtree(Type point_to_planar_thres, Type position_converge_thres,
                                    Type rotation_converge_thres, Type rot_thre_add_cloud,
                                    Type dist_thre_add_cloud, size_t local_map_size,
                                    float map_cloud_filter_size, size_t opti_iter_num,
                                    bool is_localization_mode = false) :
        point_to_planar_thres_(point_to_planar_thres), position_converge_thres_(position_converge_thres),
        rotation_converge_thres_(rotation_converge_thres), rot_thre_add_cloud_(rot_thre_add_cloud),
        dist_thre_add_cloud_(dist_thre_add_cloud), num_iter_(opti_iter_num),
        local_map_size_(local_map_size), map_cloud_filter_size_(map_cloud_filter_size),
        is_localization_mode_(is_localization_mode) {
        CHECK_NE(num_iter_, IntNaN);
        CHECK_NE(point_to_planar_thres_, std::numeric_limits<Type>::max());
        CHECK_NE(position_converge_thres_, std::numeric_limits<Type>::max());
        CHECK_NE(rotation_converge_thres_, std::numeric_limits<Type>::max());
        CHECK_NE(rot_thre_add_cloud_, std::numeric_limits<Type>::max());
        CHECK_NE(dist_thre_add_cloud_, std::numeric_limits<Type>::max());
        CHECK_NE(local_map_size_, SIZE_MAX);
        CHECK_NE(map_cloud_filter_size_, FloatNaN);

        source_cloud_ptr_.reset(new CloudType);
        local_map_ptr_.reset(new CloudType);
    }

    void AddCloudToLocalMap(const std::initializer_list<PCLPointCloudXYZI>& cloud_list) override {
        CHECK_EQ(cloud_list.size(), 1) << "LoamPointToPlane only need planar cloud";

        const auto planar_cloud = cloud_list.begin()->makeShared();

        if (is_localization_mode_) {
            local_map_ptr_ = planar_cloud;
        } else {
            cloud_deque_.push_back(planar_cloud);

            if (cloud_deque_.size() > local_map_size_) {
                cloud_deque_.pop_front();
            }

            local_map_ptr_->clear();

            for (const auto& it : cloud_deque_) {
                const auto cloud_downsample = VoxelGridCloud(it, map_cloud_filter_size_);
                *local_map_ptr_ += *it;
            }
        }

        local_map_ptr_ = VoxelGridCloud(local_map_ptr_, map_cloud_filter_size_);
        kdtree_flann_.setInputCloud(local_map_ptr_);
    }

    bool Match(const PointcloudClusterPtr& source_cloud_cluster, Mat4d& T) override {
        Timer timer;

        *source_cloud_ptr_ = source_cloud_cluster->planar_cloud_;

        const auto& planar_source = source_cloud_cluster->planar_cloud_;
        const std::size_t planar_size = source_cloud_cluster->planar_cloud_.size();
        number_planar_point_ = planar_size;
        H_planar_coeffs_.resize(planar_size);
        g_planar_coeffs_.resize(planar_size);
        planar_valid_flags_.resize(planar_size);
        res_planars_.resize(planar_size);
        T_ = T.template cast<Type>();
        bool has_converge = true;

        std::fill(planar_valid_flags_.begin(), planar_valid_flags_.end(), false);

        Type last_rotation_dx_norm = 0.0;
        Type last_position_dx_norm = 0.0;
        for (unsigned int i = 0; i < num_iter_; ++i) {
            H_.setZero();
            g_.setZero();

            PlanerMatch(planar_source);
            SumCoefficient();

            Eigen::Matrix<Type, 6, 1> dx = H_.fullPivHouseholderQr().solve(g_);
            Eigen::Matrix<Type, 3, 3> R_delta = SO3Exp(dx.head(3));
            T_.block(0, 0, 3, 3) = R_delta * T_.block(0, 0, 3, 3);
            T_.block(0, 3, 3, 1) += dx.tail(3);

            // Debug
            /*
            std::cout << "iter= " << std::setw(2) << i
                      << "  res planar= " << std::setprecision(5) << overall_res_planar_
                      << "  valid planar= " << std::setprecision(5) << number_valid_planar_
                      << "  norm dx= " << std::setprecision(5) << dx.norm() << std::endl;
            std::cout << std::setw(6) << "     \033[32m delta R = " << dx.head(3).transpose() << " | delta P = "
                      << dx.tail(3).transpose() << "\033[0m" << std::endl;
            */
            Type temp_rotation_dx_norm = dx.head(3).norm();
            Type temp_position_dx_norm = dx.tail(3).norm();
            Type delta_rotation_dx = std::fabs(temp_rotation_dx_norm - last_rotation_dx_norm);
            Type delta_position_dx = std::fabs(temp_position_dx_norm - last_position_dx_norm);
            last_rotation_dx_norm = temp_rotation_dx_norm;
            last_position_dx_norm = temp_position_dx_norm;

            if ((dx.head(3).norm() < rotation_converge_thres_ && dx.tail(3).norm() < position_converge_thres_)
                || (delta_rotation_dx < static_cast<Type>(1.0e-4) && delta_position_dx < static_cast<Type>(1.0e-4))) {
                DLOG(INFO) << "num iter= " << std::setw(2) << i
                    << "  res planar= " << std::setprecision(5) << overall_res_planar_
                    << "  all  planar = " << std::setprecision(5) << planar_source.size()
                    << "  valid planar= " << std::setprecision(5) << number_valid_planar_;
                break;
            }
        }

        T = T_.template cast<double>();
        final_transformation_ = T_;

        if (number_valid_planar_ < 50u) {
            has_converge = false;
        }

        if (has_converge && IsNeedAddCloud(final_transformation_) && !is_localization_mode_) {
            CloudType::Ptr transform_cloud(new CloudType);
            transform_cloud = TransformPointCloud(source_cloud_ptr_, final_transformation_.template cast<double>());
            AddCloudToLocalMap({*transform_cloud});
        } else if (!has_converge) {
            LOG(WARNING) << "LOAM point-to-plane match failed!\n";
        }

        DLOG(INFO) << "LoamPointToPlaneKdTree Match use time(ms): " << timer.End();

        return has_converge;
    }

    [[nodiscard]] float GetFitnessScore(float max_range) const override {
        float fitness_score = 0.0f;

        CloudType::Ptr transformed_cloud_ptr(new CloudType);
        *transformed_cloud_ptr = TransformPointCloud(*source_cloud_ptr_, final_transformation_.template cast<double>());

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
    bool IsNeedAddCloud(const Eigen::Matrix<Type, 4, 4>& T) {
        static Eigen::Matrix<Type, 4, 4> last_T = T;

        const Eigen::Matrix<Type, 3, 3> R_delta =
            last_T.template block<3, 3>(0, 0).inverse() * T.template block<3, 3>(0, 0);
        const Eigen::Matrix<Type, 3, 1> euler_delta = RotationMatrixToRPY(R_delta);

        if ((T.template block<3, 1>(0, 3) - last_T.template block<3, 1>(0, 3)).norm() > Type(dist_thre_add_cloud_) ||
            std::fabs(euler_delta.x()) > Type(rot_thre_add_cloud_) ||
            std::fabs(euler_delta.y()) > Type(rot_thre_add_cloud_) ||
            std::fabs(euler_delta.z()) > Type(rot_thre_add_cloud_)) {
            last_T = T;
            return true;
        } else {
            return false;
        }
    }

    void PlanerMatch(const PCLPointCloudXYZI& source) {
        const Eigen::Matrix<Type, 3, 3>& R = T_.block(0, 0, 3, 3);
        std::vector<size_t> indices(number_planar_point_);
        std::iota(indices.begin(), indices.end(), 0);

        std::for_each(std::execution::par_unseq, indices.begin(), indices.end(),
                      [&](const size_t& i) {
                          const auto& source_point = source.points[i];
                          PCLPointXYZI transformed_source_point =
                              pcl::transformPoint(source_point, Eigen::Transform<Type, 3, Eigen::Affine>(T_));

                          std::vector<float> dist;
                          std::vector<int> indices;
                          kdtree_flann_.nearestKSearch(transformed_source_point, 5, indices, dist);

                          if (indices.size() < 5) {
                              return;
                          }

                          Eigen::Matrix<Type, 5, 3> A; // save target planar point
                          static Eigen::Matrix<Type, 5, 1> b = Eigen::Matrix<Type, 5, 1>::Ones() * (Type(-1.0));
                          for (unsigned int j = 0; j < 5u; ++j) {
                              A.row(j) << static_cast<Type>(local_map_ptr_->points[indices[j]].x),
                                  static_cast<Type>(local_map_ptr_->points[indices[j]].y),
                                  static_cast<Type>(local_map_ptr_->points[indices[j]].z);
                          }

                          const Eigen::Matrix<Type, 3, 1>&& plane_coeff = A.colPivHouseholderQr().solve(b);
                          const Type plane_coeff_norm = plane_coeff.norm();

                          bool is_valid_plane = true;
                          for (unsigned int j = 0; j < 5u; ++j) {
                              if (std::abs(A.row(j) * plane_coeff + (Type)1.0) / plane_coeff_norm
                                  > (Type)point_to_planar_thres_) {
                                  is_valid_plane = false;
                                  break;
                              }
                          }

                          if (!is_valid_plane)
                              return;

                          const Eigen::Matrix<Type, 3, 1>&& plane_n = plane_coeff.normalized();
                          const Eigen::Matrix<Type, 3, 1> ps(static_cast<Type>(source_point.x),
                                                             static_cast<Type>(source_point.y),
                                                             static_cast<Type>(source_point.z));
                          const Eigen::Matrix<Type, 3, 1> ps_transformed(
                              static_cast<Type>(transformed_source_point.x),
                              static_cast<Type>(transformed_source_point.y),
                              static_cast<Type>(transformed_source_point.z));
                          const Type point_plane_dist = (ps_transformed - A.row(0).transpose()).dot(plane_n);

                          // Reduce the proportion of nearby points
                          if (ps.norm() < 81 * point_plane_dist * point_plane_dist) {
                              return;
                          }

                          Eigen::Matrix<Type, 6, 1> J;
                          Type temp_data = point_plane_dist > 0 ? static_cast<Type>(1) : static_cast<Type>(-1);
                          J.head(3) = -SO3Hat(R * ps).transpose() * plane_n * temp_data;
                          J.tail(3) = Eigen::Matrix<Type, 3, 3>::Identity() * plane_n * temp_data;

                          planar_valid_flags_[i] = true;
                          H_planar_coeffs_[i] = J * J.transpose();
                          g_planar_coeffs_[i] = -J * std::fabs(point_plane_dist);
                          res_planars_[i] = std::fabs(point_plane_dist);
                      }
        );
    }

    void SumCoefficient() {
        number_valid_planar_ = 0;
        overall_res_planar_ = 0;

        for (unsigned int i = 0; i < number_planar_point_; ++i) {
            if (!planar_valid_flags_[i]) {
                continue;
            }

            number_valid_planar_++;
            H_ += H_planar_coeffs_[i];
            g_ += g_planar_coeffs_[i];
            overall_res_planar_ += res_planars_[i];
        }
    }

private:
    std::vector<Eigen::Matrix<Type, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<Type, 6, 6>>> H_planar_coeffs_{};
    std::vector<Eigen::Matrix<Type, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<Type, 6, 1>>> g_planar_coeffs_{};
    std::vector<bool> planar_valid_flags_{};
    std::vector<Type> res_planars_{};

    std::deque<CloudType::Ptr> cloud_deque_;
    CloudType::Ptr local_map_ptr_ = nullptr;
    CloudType::Ptr source_cloud_ptr_ = nullptr;

    Type overall_res_planar_{std::numeric_limits<Type>::max()};
    Type point_to_planar_thres_{std::numeric_limits<Type>::max()};
    Type position_converge_thres_{std::numeric_limits<Type>::max()};
    Type rotation_converge_thres_{std::numeric_limits<Type>::max()};
    Type rot_thre_add_cloud_{std::numeric_limits<Type>::max()};
    Type dist_thre_add_cloud_{std::numeric_limits<Type>::max()};

    size_t num_iter_{SIZE_MAX};
    size_t local_map_size_{SIZE_MAX};
    size_t number_valid_planar_{};
    size_t number_planar_point_{};

    float map_cloud_filter_size_{FloatNaN};

    Eigen::Matrix<Type, 4, 4> T_{};
    Eigen::Matrix<Type, 6, 6> H_{};
    Eigen::Matrix<Type, 6, 1> g_{};

    Eigen::Matrix<Type, 4, 4> final_transformation_{};
    pcl::KdTreeFLANN<PointType> kdtree_flann_{};

    bool is_localization_mode_ = false;
};

#endif //FUNNY_LIDAR_SLAM_LOAM_POINT_TO_PLANE_KDTREE_H
