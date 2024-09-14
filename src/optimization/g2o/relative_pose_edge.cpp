//
// Created by Zhang Zhimeng on 23-11-22.
//

#include "optimization/g2o/relative_pose_edge.h"

EdgeRelativePose::EdgeRelativePose(const Mat4d &measure) {
    resize(2);
    T_measure_ = measure;
}

void EdgeRelativePose::linearizeOplus() {
    const auto *pose_vertex_i = dynamic_cast<const VertexPose *>(_vertices.at(0));
    const auto *pose_vertex_j = dynamic_cast<const VertexPose *>(_vertices.at(1));

    const Mat4d T_i = pose_vertex_i->estimate();
    const Mat4d T_j = pose_vertex_j->estimate();
    const Mat4d T_j_inv = T_j.inverse();

    const Mat4d T_error = T_measure_.inverse() * T_i.inverse() * T_j;
    const Mat6d Jr_err_inv = SE3RightJacobian(SE3Log(T_error)).inverse();
    const Mat6d T_j_inv_adj = SE3Adj(T_j_inv);

    // derivative of error with respect to T_i
    _jacobianOplus.at(0).setZero();
    _jacobianOplus.at(0) = -Jr_err_inv * T_j_inv_adj;

    // derivative of error with respect to T_j
    _jacobianOplus.at(1).setZero();
    _jacobianOplus.at(1) = Jr_err_inv * T_j_inv_adj;
}

void EdgeRelativePose::computeError() {
    const auto *pose_vertex_i = dynamic_cast<const VertexPoseType *>(_vertices.at(0));
    const auto *pose_vertex_j = dynamic_cast<const VertexPoseType *>(_vertices.at(1));

    const Mat4d T_i = pose_vertex_i->estimate();
    const Mat4d T_j = pose_vertex_j->estimate();

    const Mat4d T_error = T_measure_.inverse() * T_i.inverse() * T_j;

    // error.head(3) is position error
    // error.tail(3) is rotation error
    _error = SE3Log(T_error);
}