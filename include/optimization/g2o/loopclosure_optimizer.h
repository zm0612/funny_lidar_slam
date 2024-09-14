//
// Created by Zhang Zhimeng on 24-3-12.
//

#ifndef FUNNY_LIDAR_SLAM_LOOPCLOSURE_OPTIMIZER_H
#define FUNNY_LIDAR_SLAM_LOOPCLOSURE_OPTIMIZER_H

#include "common/keyframe.h"
#include "common/data_type.h"
#include "common/loopclosure_result.h"
#include "optimization/g2o/vertex_type.h"
#include "optimization/g2o/relative_pose_edge.h"
#include "optimization/g2o/g2o_optimizer_header.h"

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

class LoopClosureOptimizer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    using VertexType = VertexPose;
    using EdgeType = EdgeRelativePose;

public:
    LoopClosureOptimizer();

    ~LoopClosureOptimizer() = default;

    void AddVertex(const Mat4d &pose, int id, bool is_fix = false);

    void AddEdge(const Mat4d &delta_pose, const Mat6d &info, int id_i, int id_j);

    void Optimize(int num_iter = 20);

    Mat4d GetVertexEstimate(int id);

    void Save(const std::string &file_path);

private:
    std::unique_ptr<g2o::SparseOptimizer> optimizer_ptr_;
};

#endif //FUNNY_LIDAR_SLAM_LOOPCLOSURE_OPTIMIZER_H
