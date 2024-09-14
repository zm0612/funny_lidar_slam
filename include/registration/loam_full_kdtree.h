//
// Created by Zhang Zhimeng on 23-12-14.
//

#ifndef FUNNY_LIDAR_SLAM_LOAM_FULL_KDTREE_H
#define FUNNY_LIDAR_SLAM_LOAM_FULL_KDTREE_H

#include "common/timer.h"
#include "registration_interface.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <glog/logging.h>

#include <iomanip>
#include <iostream>
#include <algorithm>
#include <execution>

template <typename Type=float>
class LoamFull final : public RegistrationInterface {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LoamFull() = delete;

    LoamFull(Type point_to_planar_thres, Type point_search_thres, Type line_ratio_thres,
             Type position_converge_thres, Type rotation_converge_thres,
             Type dist_thre_add_cloud, Type rot_thre_add_cloud,
             size_t local_corner_size, size_t local_planar_size,
             float corner_voxel_filter_size, float planar_voxel_filter_size, int max_iteration) :
        point_to_planar_thres_(point_to_planar_thres), point_search_thres_(point_search_thres),
        line_ratio_thres_(line_ratio_thres), position_converge_thres_(position_converge_thres),
        rotation_converge_thres_(rotation_converge_thres), dist_thre_add_cloud_(dist_thre_add_cloud),
        rot_thre_add_cloud_(rot_thre_add_cloud), local_planar_size_(local_planar_size),
        local_corner_size_(local_corner_size), num_iter_(max_iteration) {
        CHECK_NE(num_iter_, SizeNaN);
        CHECK_NE(local_planar_size_, SizeNaN);
        CHECK_NE(local_corner_size_, SizeNaN);
        CHECK_NE(position_converge_thres_, std::numeric_limits<Type>::max());
        CHECK_NE(point_to_planar_thres_, std::numeric_limits<Type>::max());
        CHECK_NE(point_search_thres_, std::numeric_limits<Type>::max());
        CHECK_NE(rotation_converge_thres_, std::numeric_limits<Type>::max());
        CHECK_NE(line_ratio_thres_, std::numeric_limits<Type>::max());
        CHECK_NE(rot_thre_add_cloud_, std::numeric_limits<Type>::max());
        CHECK_NE(dist_thre_add_cloud_, std::numeric_limits<Type>::max());

        CHECK_GT(local_planar_size, 0) << "local planar map size must great than 0!";
        CHECK_GT(local_corner_size, 0) << "local corner map size must great than 0!";

        corner_voxel_filter_.setLeafSize(corner_voxel_filter_size, corner_voxel_filter_size, corner_voxel_filter_size);
        planar_voxel_filter_.setLeafSize(planar_voxel_filter_size, planar_voxel_filter_size, planar_voxel_filter_size);

        local_corner_cloud_ptr_.reset(new PCLPointCloudXYZI());
        local_planar_cloud_ptr_.reset(new PCLPointCloudXYZI());
    }

    /*!
     * planar corner
     * @param cloud_list
     */
    void AddCloudToLocalMap(const std::initializer_list<PCLPointCloudXYZI>& cloud_list) override {
        CHECK_EQ(cloud_list.size(), 2) << "LoamFull need corner and planar cloud";
        const auto& planar_cloud = *(cloud_list.begin());
        const auto& corner_cloud = *(cloud_list.begin() + 1);

        corner_cloud_deque_.push_back(corner_cloud);
        planar_cloud_deque_.push_back(planar_cloud);

        if (planar_cloud_deque_.size() > local_planar_size_) {
            planar_cloud_deque_.pop_front();
        }

        if (corner_cloud_deque_.size() > local_corner_size_) {
            corner_cloud_deque_.pop_front();
        }

        local_planar_cloud_ptr_->clear();
        local_corner_cloud_ptr_->clear();

        for (const auto& planar_cloud_iter : planar_cloud_deque_) {
            *local_planar_cloud_ptr_ += planar_cloud_iter;
        }

        for (const auto& corner_cloud_iter : corner_cloud_deque_) {
            *local_corner_cloud_ptr_ += corner_cloud_iter;
        }

        if (planar_cloud_deque_.size() > 5) {
            planar_voxel_filter_.setInputCloud(local_planar_cloud_ptr_);
            planar_voxel_filter_.filter(*local_planar_cloud_ptr_);
        }

        if (corner_cloud_deque_.size() > 5) {
            corner_voxel_filter_.setInputCloud(local_corner_cloud_ptr_);
            corner_voxel_filter_.filter(*local_corner_cloud_ptr_);
        }

        local_planar_kd_tree_.setInputCloud(local_planar_cloud_ptr_);
        local_corner_kd_tree_.setInputCloud(local_corner_cloud_ptr_);
    }

    bool Match(const PointcloudClusterPtr& cloud_cluster, Mat4d& T) override {
        Timer timer;

        const auto& corner_source = cloud_cluster->corner_cloud_;
        const auto& planar_source = cloud_cluster->planar_cloud_;
        const std::size_t corner_size = cloud_cluster->corner_cloud_.size();
        const std::size_t planar_size = cloud_cluster->planar_cloud_.size();

        number_corner_point_ = corner_size;
        number_planar_point_ = planar_size;
        H_corner_coeffs_.resize(corner_size);
        H_planar_coeffs_.resize(planar_size);
        g_corner_coeffs_.resize(corner_size);
        g_planar_coeffs_.resize(planar_size);
        corner_valid_flags_.resize(corner_size);
        planar_valid_flags_.resize(planar_size);
        res_corners_.resize(corner_size);
        res_planars_.resize(planar_size);

        std::fill(planar_valid_flags_.begin(), planar_valid_flags_.end(), false);
        std::fill(corner_valid_flags_.begin(), corner_valid_flags_.end(), false);

        bool has_converge = true;
        T_ = T.template cast<Type>();

        Type last_rotation_dx_norm = 0.0;
        Type last_position_dx_norm = 0.0;
        for (unsigned int i = 0; i < num_iter_; ++i) {
            H_.setZero();
            g_.setZero();

            CornerMatch(corner_source);
            PlanarMatch(planar_source);
            SumCoefficient();

            Eigen::Matrix<Type, 6, 1> dx = H_.fullPivHouseholderQr().solve(g_);
            Eigen::Matrix<Type, 3, 3> R_delta = SO3Exp(dx.head(3));

            T_.block(0, 0, 3, 3) = R_delta * T_.block(0, 0, 3, 3);
            T_.block(0, 3, 3, 1) += dx.tail(3);

            // Debug
            /*
                        std::cout << "iter= " << std::setw(2) << i
                                  << "  res corner= " << std::setprecision(5) << overall_res_corner_
                                  << "  res planar= " << std::setprecision(5) << overall_res_planar_
                                  << "  valid corner= " << std::setprecision(5) << number_valid_corner_
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
                    << "  res corner= " << std::setprecision(5) << overall_res_corner_
                    << "  res planar= " << std::setprecision(5) << overall_res_planar_
                    << "  all  corner = " << std::setprecision(5) << corner_source.size()
                    << "  valid corner= " << std::setprecision(5) << number_valid_corner_
                    << "  all  planar = " << std::setprecision(5) << planar_source.size()
                    << "  valid planar= " << std::setprecision(5) << number_valid_planar_;
                break;
            }
        }

        T = T_.template cast<double>();

        if (number_valid_planar_ < 50) {
            has_converge = false;
        }

        if (has_converge && IsNeedAddCloud(T_)) {
            PCLPointCloudXYZI transform_planar_cloud;
            PCLPointCloudXYZI transform_corner_cloud;

            pcl::transformPointCloud(cloud_cluster->planar_cloud_, transform_planar_cloud, T_);
            pcl::transformPointCloud(cloud_cluster->corner_cloud_, transform_corner_cloud, T_);

            AddCloudToLocalMap({transform_planar_cloud, transform_corner_cloud});
        } else if (!has_converge) {
            DLOG(WARNING) << "Match failed!\n"
                << "  all planar cloud size: " << planar_size
                << "  valid planar cloud size: " << number_valid_planar_ << std::endl
                << "  all corner cloud size: " << corner_size
                << "  valid corner cloud size: " << number_valid_corner_ << std::endl;
        }

        DLOG(INFO) << "LoamFull Match use time(ms)" << timer.End();

        return has_converge;
    }

    [[nodiscard]] float GetFitnessScore(float max_range) const override {
        return FloatNaN;
    }

private:
    void CornerMatch(const PCLPointCloudXYZI& source) {
        std::vector<size_t> indices(number_corner_point_);
        std::iota(indices.begin(), indices.end(), 0);
        const Eigen::Matrix<Type, 3, 3>& R = T_.template block<3, 3>(0, 0);

        std::for_each(std::execution::par, indices.begin(), indices.end(), [&](const size_t& i) {
            auto& source_point = source.points[i];

            /// TODO: use our method
            PCLPointXYZI transformed_source_point = pcl::transformPoint(source_point,
                                                                        Eigen::Transform<Type, 3, Eigen::Affine>(T_));

            std::vector<int> near_indices;
            std::vector<float> dist;
            local_corner_kd_tree_.nearestKSearch(transformed_source_point, 5, near_indices, dist);

            if (dist.back() > static_cast<Type>(point_search_thres_)) {
                return;
            }

            Eigen::Matrix<Type, 3, 5> points_matrix;

            for (unsigned int j = 0; j < 5u; ++j) {
                points_matrix.col(j) << static_cast<Type>(local_corner_cloud_ptr_->points[near_indices[j]].x),
                    static_cast<Type>(local_corner_cloud_ptr_->points[near_indices[j]].y),
                    static_cast<Type>(local_corner_cloud_ptr_->points[near_indices[j]].z);
            }

            const Eigen::Matrix<Type, 3, 1>&& center = points_matrix.rowwise().mean();
            const Eigen::Matrix<Type, 3, 5>&& points_matrix_decentralized = points_matrix.colwise() - center;
            const Eigen::Matrix<Type, 3, 3>&& variance_matrix = (points_matrix_decentralized *
                points_matrix_decentralized.transpose()) / Type(5.0);

            Eigen::JacobiSVD<Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>> svd(variance_matrix,
                Eigen::ComputeFullV);

            const Eigen::Matrix<Type, Eigen::Dynamic, 1>& singular_values = svd.singularValues();

            if (singular_values.x() <= line_ratio_thres_ * singular_values.y())
                return;

            const Eigen::Matrix<Type, 3, 3>& V = svd.matrixV();
            const Eigen::Matrix<Type, 3, 1>& corner_n = V.col(0);
            const Eigen::Matrix<Type, 3, 1> ps(static_cast<Type>(source_point.x),
                                               static_cast<Type>(source_point.y),
                                               static_cast<Type>(source_point.z));
            const Eigen::Matrix<Type, 3, 1> ps_transformed(static_cast<Type>(transformed_source_point.x),
                                                           static_cast<Type>(transformed_source_point.y),
                                                           static_cast<Type>(transformed_source_point.z));
            const Type point_line_dist = (ps_transformed - center).cross(corner_n).norm();

            Eigen::Matrix<Type, 6, 1> J;
            Eigen::Matrix<Type, 3, 1> temp_data = (ps_transformed - center).cross(corner_n) / point_line_dist;

            J.head(3) = (SO3Hat(corner_n) * SO3Hat(R * ps)).transpose() * temp_data;
            J.tail(3) = (SO3Hat(-corner_n) * Eigen::Matrix<Type, 3, 3>::Identity()).transpose() * temp_data;

            corner_valid_flags_[i] = true;
            H_corner_coeffs_[i] = J * J.transpose();
            g_corner_coeffs_[i] = -J * point_line_dist;
            res_corners_[i] = point_line_dist;
        });
    }

    void PlanarMatch(const PCLPointCloudXYZI& source) {
        const Eigen::Matrix<Type, 3, 3>& R = T_.block(0, 0, 3, 3);
        std::vector<size_t> indices(number_planar_point_);
        std::iota(indices.begin(), indices.end(), 0);

        std::for_each(std::execution::par, indices.begin(), indices.end(),
                      [&](const size_t& i) {
                          const auto& source_point = source.points[i];

                          PCLPointXYZI transformed_source_point =
                              pcl::transformPoint(source_point, Eigen::Transform<Type, 3, Eigen::Affine>(T_));

                          std::vector<float> dist;
                          std::vector<int> near_indices;
                          local_planar_kd_tree_.nearestKSearch(transformed_source_point, 5, near_indices, dist);

                          if ((Type)dist[4] > point_search_thres_) {
                              return;
                          }

                          Eigen::Matrix<Type, 5, 3> A; // save target planar point
                          static Eigen::Matrix<Type, 5, 1> b = Eigen::Matrix<Type, 5, 1>::Ones() * (Type(-1.0));
                          for (unsigned int j = 0; j < 5u; ++j) {
                              A.row(j) << static_cast<Type>(local_planar_cloud_ptr_->points[near_indices[j]].x),
                                  static_cast<Type>(local_planar_cloud_ptr_->points[near_indices[j]].y),
                                  static_cast<Type>(local_planar_cloud_ptr_->points[near_indices[j]].z);
                          }

                          const Eigen::Matrix<Type, 3, 1>&& plane_coeff = A.colPivHouseholderQr().solve(b);
                          const Type plane_coeff_norm = plane_coeff.norm();

                          bool is_valid_plane = true;
                          /// TODO: test parameter: 0.2
                          for (unsigned int j = 0; j < 5u; ++j) {
                              if (std::abs(A.row(j) * plane_coeff + static_cast<Type>(1.0)) / plane_coeff_norm
                                  > static_cast<Type>(point_to_planar_thres_)) {
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
        number_valid_corner_ = 0;
        number_valid_planar_ = 0;
        overall_res_corner_ = 0;
        overall_res_planar_ = 0;

        for (unsigned int i = 0; i < number_corner_point_; ++i) {
            if (!corner_valid_flags_[i])
                continue;

            number_valid_corner_++;
            H_ += H_corner_coeffs_[i];
            g_ += g_corner_coeffs_[i];
            overall_res_corner_ += res_corners_[i];
        }

        for (unsigned int i = 0; i < number_planar_point_; ++i) {
            if (!planar_valid_flags_[i])
                continue;

            number_valid_planar_++;
            H_ += H_planar_coeffs_[i];
            g_ += g_planar_coeffs_[i];
            overall_res_planar_ += res_planars_[i];
        }
    }

    bool IsNeedAddCloud(const Eigen::Matrix<Type, 4, 4>& T) {
        static Eigen::Matrix<Type, 4, 4> last_T = T;

        const Eigen::Matrix<Type, 3, 3> R_delta =
            last_T.template block<3, 3>(0, 0).inverse() * T.template block<3, 3>(0, 0);
        const Vec3d euler_delta = RotationMatrixToRPY(R_delta);

        if ((T.template block<3, 1>(0, 3) - last_T.template block<3, 1>(0, 3)).norm() > dist_thre_add_cloud_ ||
            std::fabs(euler_delta.x()) > rot_thre_add_cloud_ || std::fabs(euler_delta.y()) > rot_thre_add_cloud_ ||
            std::fabs(euler_delta.z()) > rot_thre_add_cloud_) {
            last_T = T;
            return true;
        }

        return false;
    }

private:
    pcl::KdTreeFLANN<PCLPointXYZI> local_corner_kd_tree_{};
    pcl::KdTreeFLANN<PCLPointXYZI> local_planar_kd_tree_{};

    PCLPointCloudXYZI::Ptr local_corner_cloud_ptr_{nullptr};
    PCLPointCloudXYZI::Ptr local_planar_cloud_ptr_{nullptr};

    std::vector<Eigen::Matrix<Type, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<Type, 6, 6>>> H_corner_coeffs_{};
    std::vector<Eigen::Matrix<Type, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<Type, 6, 6>>> H_planar_coeffs_{};
    std::vector<Eigen::Matrix<Type, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<Type, 6, 1>>> g_corner_coeffs_{};
    std::vector<Eigen::Matrix<Type, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<Type, 6, 1>>> g_planar_coeffs_{};
    std::vector<bool> corner_valid_flags_{};
    std::vector<bool> planar_valid_flags_{};

    std::vector<Type> res_planars_{};
    std::vector<Type> res_corners_{};

    Type point_to_planar_thres_{0.2};
    Type point_search_thres_{1.0};
    Type line_ratio_thres_{3.0};
    Type overall_res_planar_{0.0};
    Type overall_res_corner_{0.0};
    Type position_converge_thres_{0.01};
    Type rotation_converge_thres_{0.05};
    Type dist_thre_add_cloud_{1.0};
    Type rot_thre_add_cloud_{0.2};

    size_t number_valid_corner_{0u};
    size_t number_valid_planar_{0u};
    size_t number_planar_point_{0u};
    size_t number_corner_point_{0u};
    size_t local_planar_size_{30u};
    size_t local_corner_size_{30u};
    size_t num_iter_{30u};

    Eigen::Matrix<Type, 4, 4> T_{};
    Eigen::Matrix<Type, 6, 6> H_{};
    Eigen::Matrix<Type, 6, 1> g_{};

    std::deque<PCLPointCloudXYZI> corner_cloud_deque_;
    std::deque<PCLPointCloudXYZI> planar_cloud_deque_;

    pcl::VoxelGrid<PCLPointXYZI> corner_voxel_filter_;
    pcl::VoxelGrid<PCLPointXYZI> planar_voxel_filter_;
};

#endif //FUNNY_LIDAR_SLAM_LOAM_FULL_KDTREE_H
