//
// Created by Zhang Zhimeng on 24-3-20.
//
#include "optimization/g2o/loopclosure_optimizer.h"

#include <gtest/gtest.h>
#include <random>

static std::string file_path("");

struct Data {
    Mat4d pose_true = Mat4d::Identity();
    Mat4d pose_noise = Mat4d::Identity();
};

std::vector<Data> GenData(unsigned int data_size) {
    double delta_yaw = M_PI * 2.0 / data_size;
    double yaw_noise = 5.0 / 180.0 * M_PI;
    double circle_radius = 10.0;

    std::random_device rd;
    std::default_random_engine eng(rd());
    std::normal_distribution<double> pt_noise(0.0, 1.0);
    std::normal_distribution<double> yaw_nd(0.0, yaw_noise);

    std::vector<Data> data_all;

    for (unsigned int i = 0; i < data_size; ++i) {
        double yaw = i * delta_yaw;

        Mat4d pose_noise = Mat4d::Identity();
        pose_noise(0, 3) = circle_radius * cos(yaw) + pt_noise(eng);
        pose_noise(1, 3) = circle_radius * sin(yaw) + pt_noise(eng);
        pose_noise.block<3, 3>(0, 0) = Eigen::AngleAxisd(yaw + +yaw_nd(eng), Vec3d::UnitZ()).toRotationMatrix();

        Mat4d pose_true = Mat4d::Identity();
        pose_true(0, 3) = circle_radius * cos(yaw);
        pose_true(1, 3) = circle_radius * sin(yaw);
        pose_true.block<3, 3>(0, 0) = Eigen::AngleAxisd(yaw, Vec3d::UnitZ()).toRotationMatrix();

        Data data;
        data.pose_true = pose_true;
        data.pose_noise = pose_noise;

        data_all.push_back(data);
    }

    return data_all;
}

TEST(TestLoopClosureOptimizer, TestCircle) {
    const auto data_all = GenData(60);

    LoopClosureOptimizer loop_closure_optimizer;

    for (int i = 0; i < static_cast<int>(data_all.size()); ++i) {
        if (i == 0) {
            loop_closure_optimizer.AddVertex(data_all[i].pose_true, i, true);
        } else {
            loop_closure_optimizer.AddVertex(data_all[i].pose_noise, i, false);
        }

        if (i >= 1) {
            Mat4d delta_pose = data_all[i - 1].pose_true.inverse() * data_all[i].pose_true;
            loop_closure_optimizer.AddEdge(delta_pose, Mat6d::Identity(), i - 1, i);
        }
    }

    Mat4d delta_pose =
            data_all[0].pose_true.inverse() * data_all[static_cast<int>(data_all.size()) - 10].pose_true;
    loop_closure_optimizer.AddEdge(delta_pose, Mat6d::Identity(), 0, static_cast<int>(data_all.size()) - 10);

    loop_closure_optimizer.Optimize(10);

    if (!file_path.empty()) {
        std::ofstream optimized_path_file(file_path + "path.txt", std::ios::trunc);
        std::ofstream noise_path_file(file_path + "path_noise.txt", std::ios::trunc);

        for (unsigned int i = 0; i < data_all.size(); ++i) {
            Mat4d pose = loop_closure_optimizer.GetVertexEstimate(static_cast<int>(i));

            optimized_path_file << pose(0, 3) << " " << pose(1, 3) << std::endl;
            noise_path_file << data_all[i].pose_noise(0, 3) << " " << data_all[i].pose_noise(1, 3) << std::endl;
        }
    }

    for (unsigned int i = 0; i < data_all.size(); ++i) {
        const Mat4d true_pose = data_all[i].pose_true;
        const Mat4d opti_pose = loop_closure_optimizer.GetVertexEstimate(static_cast<int>(i));
        EXPECT_TRUE(true_pose.isApprox(opti_pose, 1.0e-4));
    }
}