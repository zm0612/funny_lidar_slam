//
// Created by Zhang Zhimeng on 23-12-14.
//

#ifndef FUNNY_LIDAR_SLAM_LOAM_POINT_TO_PLANE_IVOX_H
#define FUNNY_LIDAR_SLAM_LOAM_POINT_TO_PLANE_IVOX_H

#include "ivox_map/ivox_map.h"
#include "common/timer.h"
#include "common/data_type.h"
#include "common/constant_variable.h"
#include "common/pointcloud_utility.h"
#include "registration/registration_interface.h"

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <glog/logging.h>

#include <iomanip>
#include <iostream>
#include <algorithm>
#include <execution>

/*!
 * @brief Point to Plane LOAM registration
 * @note Not recommended for use in localization mode
 */
template <typename Type=float>
class LoamPointToPlaneIVOX final : public RegistrationInterface {
public:
    using PointType = PCLPointXYZI;
    using CloudType = PCLPointCloudXYZI;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit LoamPointToPlaneIVOX(Type point_to_planar_thres, Type position_converge_thres,
                                  Type rotation_converge_thres, size_t opti_iter_num = 30u,
                                  bool is_localization_mode = false) :
        point_to_planar_thres_(point_to_planar_thres),
        position_converge_thres_(position_converge_thres),
        rotation_converge_thres_(rotation_converge_thres),
        num_iter_(opti_iter_num), is_localization_mode_(is_localization_mode),
        source_cloud_ptr_(new CloudType) {
        CHECK_NE(num_iter_, IntNaN);
        CHECK_NE(position_converge_thres_, std::numeric_limits<Type>::max());
        CHECK_NE(point_to_planar_thres_, std::numeric_limits<Type>::max());
        CHECK_NE(rotation_converge_thres_, std::numeric_limits<Type>::max());
        InitIVox();
    }

    void InitIVox() {
        IVoxMap::Options options;
        options.resolution_ = 0.5;
        options.nearby_type_ = IVoxMap::NearbyType::NEARBY18;
        ivox_map_ptr_ = std::make_shared<IVoxMap>(options);
    }

    void AddCloudToLocalMap(const std::initializer_list<PCLPointCloudXYZI> &cloud_list) override {
        CHECK_EQ(cloud_list.size(), 1) << "LoamPointToPlane only need planar cloud";
        static bool is_first = true;

        if (is_localization_mode_) {
            Timer timer_init_ivox;
            is_first = true;
            InitIVox();
            DLOG(INFO) << "Init ivox use time: " << timer_init_ivox.End();
        }

        const auto planar_cloud = *(cloud_list.begin());

        if (is_first) {
            Timer timer_add_points;
            ivox_map_ptr_->AddPoints(planar_cloud.points);
            DLOG(INFO) << "Add points to ivox use time: " << timer_add_points.End();
            is_first = false;
        } else {
            IVoxMap::PointVector points_to_add;
            IVoxMap::PointVector point_no_need_downsample;

            points_to_add.reserve(number_planar_point_);
            point_no_need_downsample.reserve(number_planar_point_);

            std::vector<size_t> index(number_planar_point_);
            for (size_t i = 0; i < number_planar_point_; ++i) {
                index[i] = i;
            }

            std::for_each(std::execution::unseq, index.begin(), index.end(), [&](const size_t &i) {
                const PCLPointXYZI &point_world = pcl::transformPoint(planar_cloud.points[i],
                                                                      Eigen::Transform<Type, 3, Eigen::Affine>(T_));

                if (!nearest_points_[i].empty()) {
                    const IVoxMap::PointVector &points_near = nearest_points_[i];

                    Eigen::Matrix<Type, 3, 1> center =
                    ((point_world.getVector3fMap().cast<Type>() / filter_size_map_min_).array().floor() +
                        static_cast<Type>(0.5)) * filter_size_map_min_;

                    Eigen::Matrix<Type, 3, 1> dis_2_center = points_near[0].getVector3fMap().cast<Type>() - center;

                    if (std::fabs(dis_2_center.x()) > static_cast<Type>(0.5) * filter_size_map_min_ &&
                        std::fabs(dis_2_center.y()) > static_cast<Type>(0.5) * filter_size_map_min_ &&
                        std::fabs(dis_2_center.z()) > static_cast<Type>(0.5) * filter_size_map_min_) {
                        point_no_need_downsample.emplace_back(point_world);
                        return;
                    }

                    bool need_add = true;
                    Type dist = (point_world.getVector3fMap().cast<Type>() - center).squaredNorm();
                    if (points_near.size() >= 5u) {
                        for (int readd_i = 0; readd_i < 5; readd_i++) {
                            if ((points_near[readd_i].getVector3fMap().cast<Type>() - center).squaredNorm()
                                < dist + 1.0e-6) {
                                need_add = false;
                                break;
                            }
                        }
                    }

                    if (need_add) {
                        points_to_add.emplace_back(point_world);
                    }
                } else {
                    points_to_add.emplace_back(point_world);
                }
            });

            ivox_map_ptr_->AddPoints(points_to_add);
            ivox_map_ptr_->AddPoints(point_no_need_downsample);
        }

        if (is_localization_mode_) {
            Timer timer_build_kdtree;
            kdtree_flann_.setInputCloud(planar_cloud.makeShared());
            DLOG(INFO) << "Build kdtree use time: " << timer_build_kdtree.End();
        }
    }

    bool Match(const PointcloudClusterPtr &source_cloud_cluster, Mat4d &T) override {
        Timer timer_match;

        *source_cloud_ptr_ = source_cloud_cluster->planar_cloud_;

        const auto &planar_source = source_cloud_cluster->planar_cloud_;
        const std::size_t planar_size = source_cloud_cluster->planar_cloud_.size();
        number_planar_point_ = planar_size;
        H_planar_coeffs_.resize(planar_size);
        g_planar_coeffs_.resize(planar_size);
        planar_valid_flags_.resize(planar_size);
        res_planars_.resize(planar_size);
        T_ = T.cast<Type>();
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

        if (has_converge && !is_localization_mode_) {
            AddCloudToLocalMap({planar_source});
        } else if (!has_converge) {
            DLOG(WARNING) << "Match failed!" << std::endl
                << "  all planar cloud size: " << planar_size
                << "  valid planar cloud size: " << number_valid_planar_ << std::endl;
        }

        DLOG(INFO) << "LoamPointToPlane Match use time(ms): " << timer_match.End();

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
        *transformed_cloud_ptr = TransformPointCloud(*source_cloud_ptr_,
                                                     final_transformation_.template cast<double>());

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
    void PlanerMatch(const PCLPointCloudXYZI &source) {
        nearest_points_.resize(number_planar_point_);
        const Eigen::Matrix<Type, 3, 3> &R = T_.block(0, 0, 3, 3);
        std::vector<size_t> indices(number_planar_point_);
        std::iota(indices.begin(), indices.end(), 0);

        std::for_each(std::execution::par_unseq, indices.begin(), indices.end(),
                      [&](const size_t &i) {
                          const auto &source_point = source.points[i];
                          PCLPointXYZI transformed_source_point =
                              pcl::transformPoint(source_point, Eigen::Transform<Type, 3, Eigen::Affine>(T_));

                          IVoxMap::PointVector &point_vector = nearest_points_[i];
                          ivox_map_ptr_->GetClosestPoint(transformed_source_point, point_vector, 5);

                          if (point_vector.size() < 5) {
                              return;
                          }

                          Eigen::Matrix<Type, 5, 3> A; // save target planar point
                          static Eigen::Matrix<Type, 5, 1> b = Eigen::Matrix<Type, 5, 1>::Ones() * Type(-1.0);
                          for (unsigned int j = 0; j < 5u; ++j) {
                              A.row(j) << static_cast<Type>(point_vector[j].x),
                                  static_cast<Type>(point_vector[j].y),
                                  static_cast<Type>(point_vector[j].z);
                          }

                          const Eigen::Matrix<Type, 3, 1> &&plane_coeff = A.colPivHouseholderQr().solve(b);
                          const Type plane_coeff_norm = plane_coeff.norm();

                          bool is_valid_plane = true;
                          for (unsigned int j = 0; j < 5u; ++j) {
                              if (std::abs(A.row(j) * plane_coeff + static_cast<Type>(1.0)) / plane_coeff_norm
                                  > static_cast<Type>(point_to_planar_thres_)) {
                                  is_valid_plane = false;
                                  break;
                              }
                          }

                          if (!is_valid_plane)
                              return;

                          const Eigen::Matrix<Type, 3, 1> &&plane_n = plane_coeff.normalized();
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
    std::vector<PCLPointVector> nearest_points_;
    std::vector<bool> planar_valid_flags_{};
    std::vector<Type> res_planars_{};

    Type overall_res_planar_{};
    Type point_to_planar_thres_{0.2};
    Type filter_size_map_min_{0.5};
    Type position_converge_thres_{0.01};
    Type rotation_converge_thres_{0.05};

    size_t num_iter_{30u};
    size_t number_valid_planar_{};
    size_t number_planar_point_{};

    Eigen::Matrix<Type, 4, 4> T_{};
    Eigen::Matrix<Type, 6, 6> H_{};
    Eigen::Matrix<Type, 6, 1> g_{};

    std::shared_ptr<IVoxMap> ivox_map_ptr_ = nullptr;

    Eigen::Matrix<Type, 4, 4> final_transformation_{};
    pcl::KdTreeFLANN<PointType> kdtree_flann_{};
    bool is_localization_mode_ = false;
    CloudType::Ptr source_cloud_ptr_ = nullptr;
};

#endif //FUNNY_LIDAR_SLAM_LOAM_POINT_TO_PLANE_IVOX_H
