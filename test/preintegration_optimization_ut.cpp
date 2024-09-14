//
// Created by Zhang Zhimeng on 23-10-26.
//

#include "imu/pre_integration.h"
#include "optimization/g2o/vertex_type.h"
#include "optimization/g2o/position_edge.h"
#include "optimization/g2o/rotation_edge.h"
#include "optimization/g2o/gyro_bias_rw_edge.h"
#include "optimization/g2o/accel_bias_rw_edge.h"
#include "optimization/g2o/prior_nav_state_edge.h"
#include "optimization/g2o/pre_integration_edge.h"
#include "optimization/g2o/g2o_optimizer_header.h"
#include "imu/data_synchronizer.h"

#include <random>
#include <gtest/gtest.h>

class PreIntegrationOptimizationTest {
public:
    PreIntegrationOptimizationTest() = default;

    // 生成圆弧运动时的IMU数据和GPS数据
    void GenImuData(std::vector<IMUData> &imu_data_vec, std::vector<GPSData> &gps_data_vec,
                    std::vector<NavStateData> &nav_state_gt_data_vec) const {
        // 噪声生成器
        std::random_device rd;
        std::default_random_engine eng(rd());
        std::normal_distribution<double> acc_normal_noise(0.0, imu_acc_noise_std_);
        std::normal_distribution<double> gyro_normal_noise(0.0, imu_gyro_noise_std_);
        std::normal_distribution<double> gps_posi_normal_noise(0.0, gps_posi_noise_std_);
        std::normal_distribution<double> gps_vel_normal_noise(0.0, gps_velocity_noise_std_);
        std::normal_distribution<double> gps_ori_normal_noise(0.0, gps_orientation_noise_std_);

        // true acceleration
        double acceleration_true = omega_true_ * velocity_true_;

        imu_data_vec.clear();
        gps_data_vec.clear();
        nav_state_gt_data_vec.clear();

        // Calculate total simulation data volume based on total time and step time
        int data_size = static_cast<int>(sum_time_ / imu_dt_);

        nav_state_gt_data_vec.reserve(data_size);
        imu_data_vec.reserve(data_size);
        gps_data_vec.reserve(data_size);

        NavStateData last_nav_state_data;
        last_nav_state_data.P_.setZero();
        last_nav_state_data.V_ << 0.0, velocity_true_, 0.0;
        last_nav_state_data.R_.setIdentity();

        for (int i = 0; i < data_size; i++) {
            Eigen::Vector3d acceleration;
            acceleration << acceleration_true, 0.0, 0.0;

            Eigen::Vector3d angular_velocity;
            angular_velocity << 0.0, 0.0, -omega_true_;

            // 生成IMU的仿真数据量
            IMUData imu_data;
            imu_data.linear_acceleration_
                    << acceleration_true + imu_acc_bias_ + acc_normal_noise(eng),
                    imu_acc_bias_ + acc_normal_noise(eng),
                    imu_acc_bias_ + acc_normal_noise(eng);
            imu_data.angular_velocity_
                    << angular_velocity + Vec3d(imu_gyro_bias_ + gyro_normal_noise(eng),
                                                imu_gyro_bias_ + gyro_normal_noise(eng),
                                                imu_gyro_bias_ + gyro_normal_noise(eng));
            imu_data.timestamp_ = static_cast<uint64_t>(i * 0.01 * 1.0e6);
            imu_data.orientation_ = last_nav_state_data.R_;
            imu_data_vec.emplace_back(std::move(imu_data));

            // 生成GPS的测量数据
            if (i % 10 == 0) {
                GPSData gps_data;

                gps_data.local_xyz_.x() = last_nav_state_data.P_.x() + gps_posi_normal_noise(eng);
                gps_data.local_xyz_.y() = last_nav_state_data.P_.y() + gps_posi_normal_noise(eng);
                gps_data.local_xyz_.z() = last_nav_state_data.P_.z() + gps_posi_normal_noise(eng);
                gps_data.local_xyz_velocity_.x() = last_nav_state_data.V_.x() + gps_vel_normal_noise(eng);
                gps_data.local_xyz_velocity_.y() = last_nav_state_data.V_.y() + gps_vel_normal_noise(eng);
                gps_data.local_xyz_velocity_.z() = last_nav_state_data.V_.z() + gps_vel_normal_noise(eng);

                Vec3d ori_noise(gps_ori_normal_noise(eng),
                                gps_ori_normal_noise(eng),
                                gps_ori_normal_noise(eng));

                gps_data.local_orientation = last_nav_state_data.R_ * SO3Exp(ori_noise);

                gps_data.timestamp_ = static_cast<uint64_t>(i * imu_dt_ * 1.0e6);

                gps_data_vec.emplace_back(std::move(gps_data));
            }
            last_nav_state_data.timestamp_ = static_cast<uint64_t>(i * imu_dt_ * 1.0e6);
            nav_state_gt_data_vec.push_back(last_nav_state_data);

            // 生成真实运动轨迹
            last_nav_state_data.P_ = last_nav_state_data.P_ + last_nav_state_data.V_ * imu_dt_
                                     + 0.5 * imu_dt_ * imu_dt_ * last_nav_state_data.R_ * acceleration;
            last_nav_state_data.V_ = last_nav_state_data.V_ + last_nav_state_data.R_ * acceleration * imu_dt_;
            last_nav_state_data.R_ = last_nav_state_data.R_ * SO3Exp(angular_velocity * imu_dt_);
        }
    }

public:
    double sum_time_ = 60.0; // 总仿真时间
    double imu_dt_ = 0.01; // imu采样时间

    double imu_acc_noise_std_ = 0.01;
    double imu_acc_bias_ = 0.2;

    double imu_gyro_noise_std_ = 0.01;
    double imu_gyro_bias_ = 0.1;

    double imu_bias_rw_noise_std_ = 0.001; // imu bias随机游走的标准差

    double gps_posi_noise_std_ = 1.0;
    double gps_velocity_noise_std_ = 0.1;
    double gps_orientation_noise_std_ = 1.0 * kDegree2Radian; // degree

    double velocity_true_ = 5.0; // velocity norm
    double omega_true_ = 5.0 * kDegree2Radian;
};

TEST(TestPreIntegrationOptimization, TestErrorCompute) {
    std::vector<IMUData> imu_data_vec;
    std::vector<GPSData> gps_data_vec;
    std::vector<NavStateData> nav_state_data_vec;
    PreIntegrationOptimizationTest pre_integration_optimization_test;
    pre_integration_optimization_test.GenImuData(imu_data_vec, gps_data_vec, nav_state_data_vec);

    double gyro_sigma = pre_integration_optimization_test.imu_gyro_noise_std_;
    double accel_sigma = pre_integration_optimization_test.imu_acc_noise_std_;

    PreIntegration::ConfigPara config_para;
    Vec3d gravity(0.0, 0.0, 0.0);
    config_para.gravity_ = gravity;
    config_para.gyro_noise_std_ << gyro_sigma, gyro_sigma, gyro_sigma;
    config_para.acc_noise_std_ << accel_sigma, accel_sigma, accel_sigma;
    config_para.init_acc_bias_ << 0.0, 0.0, 0.0;
    config_para.init_gyro_bias_ << 0.0, 0.0, 0.0;
    config_para.integration_noise_cov_ << 1.0e-8, 1.0e-8, 1.0e-8;

    NavStateData last_nav_state;
    last_nav_state.P_ = nav_state_data_vec.front().P_;
    last_nav_state.R_ = nav_state_data_vec.front().R_;
    last_nav_state.V_ = nav_state_data_vec.front().V_;

    double prior_rotation_noise_std = 1.0e-5;
    double prior_velocity_noise_std = 1.0e-5;
    double prior_position_noise_std = 5.0e-4;
    double prior_gyro_bias_noise_std = pre_integration_optimization_test.imu_gyro_bias_;
    double prior_acc_bias_noise_std = pre_integration_optimization_test.imu_acc_bias_;
    Mat15d last_nav_state_info = Mat15d::Identity();
    last_nav_state_info.block<3, 3>(0, 0) *= 1.0 / std::pow(prior_rotation_noise_std, 2.0);
    last_nav_state_info.block<3, 3>(3, 3) *= 1.0 / std::pow(prior_velocity_noise_std, 2.0);
    last_nav_state_info.block<3, 3>(6, 6) *= 1.0 / std::pow(prior_position_noise_std, 2.0);
    last_nav_state_info.block<3, 3>(9, 9) *= 1.0 / std::pow(prior_gyro_bias_noise_std, 2.0);
    last_nav_state_info.block<3, 3>(12, 12) *= 1.0 / std::pow(prior_acc_bias_noise_std, 2.0);

    DataSynchronizer pre_integration_data_preparer;
    for (const auto &i: imu_data_vec) {
        pre_integration_data_preparer.CacheData(i);
    }

    std::string home_path = getenv("HOME");
    std::ofstream fuse_path_file(home_path + "/fuse_path.txt", std::ios::trunc);
    std::ofstream imu_bias_file(home_path + "/imu_bias.txt", std::ios::trunc);
    std::ofstream gt_file(home_path + "/gt_path.txt", std::ios::trunc);
    std::ofstream gps_file(home_path + "/gps_measure.txt", std::ios::trunc);

    imu_bias_file << pre_integration_optimization_test.imu_acc_bias_ << " "
                  << pre_integration_optimization_test.imu_acc_bias_ << " "
                  << pre_integration_optimization_test.imu_acc_bias_ << " "
                  << pre_integration_optimization_test.imu_gyro_bias_ << " "
                  << pre_integration_optimization_test.imu_gyro_bias_ << " "
                  << pre_integration_optimization_test.imu_gyro_bias_ << std::endl;

    for (const auto &gt_data: nav_state_data_vec) {
        Eigen::Quaterniond q(gt_data.R_);
        gt_file << std::setprecision(15) << static_cast<double >(gt_data.timestamp_) / 1.0e6 << " "
                << gt_data.P_.x() << " " << gt_data.P_.y() << " " << gt_data.P_.z() << " "
                << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
                << std::endl;
    }

    for (const auto &gps_data: gps_data_vec) {
        Eigen::Quaterniond q(gps_data.local_orientation);
        gps_file << std::setprecision(15) << static_cast<double >(gps_data.timestamp_) / 1.0e6 << " "
                 << gps_data.local_xyz_.x() << " " << gps_data.local_xyz_.y() << " " << gps_data.local_xyz_.z() << " "
                 << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
                 << std::endl;
    }

    for (unsigned int i = 0; i < gps_data_vec.size() - 1; ++i) {
        auto pre_integration_ptr = std::make_shared<PreIntegration>(config_para);

        const auto imu_data_segment = pre_integration_data_preparer.GetDataSegment(gps_data_vec[i].timestamp_,
                                                                                   gps_data_vec[i + 1].timestamp_);
        pre_integration_ptr->IntegrateDataSegment(imu_data_segment);
        NavStateData curr_nav_state = pre_integration_ptr->Predict(last_nav_state);

        using BlockSolverType = g2o::BlockSolverX;
        using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;
        auto *solver = new g2o::OptimizationAlgorithmLevenberg(
                std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>())
        );

        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        auto vertex_rotation_last = new VertexRotation;
        vertex_rotation_last->setId(0);
        vertex_rotation_last->setEstimate(last_nav_state.R_);
        optimizer.addVertex(vertex_rotation_last);

        auto vertex_velocity_last = new VertexVelocity;
        vertex_velocity_last->setId(1);
        vertex_velocity_last->setEstimate(last_nav_state.V_);
        optimizer.addVertex(vertex_velocity_last);

        auto vertex_position_last = new VertexPosition;
        vertex_position_last->setId(2);
        vertex_position_last->setEstimate(last_nav_state.P_);
        optimizer.addVertex(vertex_position_last);

        auto vertex_rotation_curr = new VertexRotation;
        vertex_rotation_curr->setId(3);
        vertex_rotation_curr->setEstimate(curr_nav_state.R_);
        optimizer.addVertex(vertex_rotation_curr);

        auto vertex_velocity_curr = new VertexVelocity;
        vertex_velocity_curr->setId(4);
        vertex_velocity_curr->setEstimate(curr_nav_state.V_);
        optimizer.addVertex(vertex_velocity_curr);

        auto vertex_position_curr = new VertexPosition;
        vertex_position_curr->setId(5);
        vertex_position_curr->setEstimate(curr_nav_state.P_);
        optimizer.addVertex(vertex_position_curr);

        auto vertex_bg_last = new VertexBiasGyro;
        vertex_bg_last->setId(6);
        vertex_bg_last->setEstimate(last_nav_state.bg_);
        optimizer.addVertex(vertex_bg_last);

        auto vertex_ba_last = new VertexBiasAccel;
        vertex_ba_last->setId(7);
        vertex_ba_last->setEstimate(last_nav_state.ba_);
        optimizer.addVertex(vertex_ba_last);

        // current gyro bias vertex
        auto vertex_bg_curr = new VertexBiasGyro;
        vertex_bg_curr->setId(8);
        vertex_bg_curr->setEstimate(last_nav_state.bg_);
        optimizer.addVertex(vertex_bg_curr);

        // current accel bias vertex
        auto vertex_ba_curr = new VertexBiasAccel;
        vertex_ba_curr->setId(9);
        vertex_ba_curr->setEstimate(last_nav_state.ba_);
        optimizer.addVertex(vertex_ba_curr);

        // === prior last nav state edge ===
        auto *edge_prior_last_nav_state = new EdgePriorNavState(last_nav_state, last_nav_state_info);
        edge_prior_last_nav_state->setVertex(0, vertex_rotation_last);
        edge_prior_last_nav_state->setVertex(1, vertex_velocity_last);
        edge_prior_last_nav_state->setVertex(2, vertex_position_last);
        edge_prior_last_nav_state->setVertex(3, vertex_bg_last);
        edge_prior_last_nav_state->setVertex(4, vertex_ba_last);
        optimizer.addEdge(edge_prior_last_nav_state);

        // set gps info
        double gps_posi_noise = pre_integration_optimization_test.gps_posi_noise_std_;
        double gps_rot_noise = pre_integration_optimization_test.gps_orientation_noise_std_;
        Mat3d gps_posi_info = Mat3d::Identity() * 1 / std::pow(gps_posi_noise, 2.0);
        Mat3d gps_rot_info = Mat3d::Identity() * 1 / std::pow(gps_rot_noise, 2.0);

        // === current lidar rotation edge ===
        auto *edge_gps_rotation_edge = new EdgeRotation();
        edge_gps_rotation_edge->setVertex(0, vertex_rotation_curr);
        edge_gps_rotation_edge->setInformation(gps_rot_info);
        edge_gps_rotation_edge->setMeasurement(gps_data_vec[i + 1].local_orientation);
        optimizer.addEdge(edge_gps_rotation_edge);

        // === current lidar position edge ===
        auto *edge_gps_position_edge = new EdgePosition();
        edge_gps_position_edge->setVertex(0, vertex_position_curr);
        edge_gps_position_edge->setInformation(gps_posi_info);
        edge_gps_position_edge->setMeasurement(gps_data_vec[i + 1].local_xyz_);
        optimizer.addEdge(edge_gps_position_edge);

        auto edge_preintegration = new EdgePreIntegration(pre_integration_ptr);
        edge_preintegration->setVertex(0, vertex_rotation_last);
        edge_preintegration->setVertex(1, vertex_velocity_last);
        edge_preintegration->setVertex(2, vertex_position_last);
        edge_preintegration->setVertex(3, vertex_bg_last);
        edge_preintegration->setVertex(4, vertex_ba_last);
        edge_preintegration->setVertex(5, vertex_rotation_curr);
        edge_preintegration->setVertex(6, vertex_velocity_curr);
        edge_preintegration->setVertex(7, vertex_position_curr);
        optimizer.addEdge(edge_preintegration);

        Mat3d acc_bias_rw_info = Mat3d::Identity();
        acc_bias_rw_info = acc_bias_rw_info
                           * (1.0 / std::pow(pre_integration_optimization_test.imu_bias_rw_noise_std_, 2.0));

        // === imu acc bias random walk edge ===
        auto *edge_acc_bias_rw = new EdgeAccelBiasRW();
        edge_acc_bias_rw->setVertex(0, vertex_ba_last);
        edge_acc_bias_rw->setVertex(1, vertex_ba_curr);
        edge_acc_bias_rw->setInformation(acc_bias_rw_info);
        optimizer.addEdge(edge_acc_bias_rw);

        // === imu gyro bias random walk edge ===
        Mat3d gyro_bias_rw_info = Mat3d::Identity();
        gyro_bias_rw_info = gyro_bias_rw_info
                            * (1.0 / std::pow(pre_integration_optimization_test.imu_bias_rw_noise_std_, 2.0));
        auto *edge_gyro_bias_rw = new EdgeGyroBiasRW();
        edge_gyro_bias_rw->setVertex(0, vertex_bg_last);
        edge_gyro_bias_rw->setVertex(1, vertex_bg_curr);
        edge_gyro_bias_rw->setInformation(gyro_bias_rw_info);
        optimizer.addEdge(edge_gyro_bias_rw);

//        optimizer.setVerbose(true);
        optimizer.initializeOptimization();
        optimizer.optimize(30);

        // Get all state posterior information
        // R_i, V_i, P_i, bg_i, ba_i, R_j, V_j, P_j, bg_j, ba_j
        //  0    3    6    9    12    15    18   21   24    27
        Eigen::Matrix<double, 30, 30> posterior_info = Eigen::Matrix<double, 30, 30>::Zero();

        // gyro bias random walk posterior info
        const Mat6d gyro_bias_posterior_info = edge_gyro_bias_rw->GetPosteriorInformation();
        posterior_info.block<3, 3>(9, 9) += gyro_bias_posterior_info.block<3, 3>(0, 0);
        posterior_info.block<3, 3>(24, 24) += gyro_bias_posterior_info.block<3, 3>(3, 3);
        posterior_info.block<3, 3>(9, 24) += gyro_bias_posterior_info.block<3, 3>(0, 3);
        posterior_info.block<3, 3>(24, 9) += gyro_bias_posterior_info.block<3, 3>(3, 0);

        // acc bias random walk posterior info
        const Mat6d acc_bias_posterior_info = edge_acc_bias_rw->GetPosteriorInformation();
        posterior_info.block<3, 3>(12, 12) += acc_bias_posterior_info.block<3, 3>(0, 0);
        posterior_info.block<3, 3>(27, 27) += acc_bias_posterior_info.block<3, 3>(3, 3);
        posterior_info.block<3, 3>(12, 27) += acc_bias_posterior_info.block<3, 3>(0, 3);
        posterior_info.block<3, 3>(27, 12) += acc_bias_posterior_info.block<3, 3>(3, 0);

        // pre-integration posterior info
        const Mat24d edge_preintegration_posterior_info = edge_preintegration->GetPosteriorInformation();
        posterior_info.block<24, 24>(0, 0) += edge_preintegration_posterior_info;

        // lidar rotation posterior info
        const Mat3d edge_gps_rotation_posterior_info = edge_gps_rotation_edge->GetPosteriorInformation();
        posterior_info.block<3, 3>(15, 15) += edge_gps_rotation_posterior_info;

        // lidar position posterior info
        const Mat3d edge_gps_position_posterior_info = edge_gps_position_edge->GetPosteriorInformation();
        posterior_info.block<3, 3>(21, 21) += edge_gps_position_posterior_info;

        // last nav state prior posterior info
        const Mat15d edge_last_nav_state_prior_posterior_info = edge_prior_last_nav_state->GetPosteriorInformation();
        posterior_info.block<15, 15>(0, 0) += edge_last_nav_state_prior_posterior_info;

        last_nav_state.P_ = vertex_position_curr->estimate();
        last_nav_state.R_ = vertex_rotation_curr->estimate();
        last_nav_state.V_ = vertex_velocity_curr->estimate();
        last_nav_state.ba_ = vertex_ba_curr->estimate();
        last_nav_state.bg_ = vertex_bg_curr->estimate();
        last_nav_state.timestamp_ = curr_nav_state.timestamp_;

        last_nav_state_info = Marginalize(posterior_info, 0, 14).block<15, 15>(15, 15);

        Eigen::Quaterniond fusion_q(last_nav_state.R_);
        fuse_path_file << std::setprecision(15) << static_cast<double >(gps_data_vec[i].timestamp_) / 1.0e6 << " "
                       << last_nav_state.P_.x() << " " << last_nav_state.P_.y() << " " << last_nav_state.P_.z() << " "
                       << fusion_q.x() << " " << fusion_q.y() << " " << fusion_q.z() << " " << fusion_q.w()
                       << std::endl;

        imu_bias_file << last_nav_state.ba_.x() << " "
                      << last_nav_state.ba_.y() << " "
                      << last_nav_state.ba_.z() << " "
                      << last_nav_state.bg_.x() << " "
                      << last_nav_state.bg_.y() << " "
                      << last_nav_state.bg_.z() << std::endl;
    }

    // Note:
    // 融合的轨迹保存在了家目录下
    // 使用evo工具进行精度评估，参考命令: evo_traj tum gps_measure.txt fuse_path.txt -p --plot_mode=xy
    // 也可使用脚本进行效果演示，参考命令：python display_gps_imu_fusion_result.py
}