//
// Created by Zhang Zhimeng on 24-1-17.
//
#include "optimization/g2o/accel_bias_rw_edge.h"

void EdgeAccelBiasRW::computeError() {
    const auto *vertex_bias_accel_0 = dynamic_cast<const VertexBiasAccel *>(_vertices[0]);
    const auto *vertex_bias_accel_1 = dynamic_cast<const VertexBiasAccel *>(_vertices[1]);
    _error = vertex_bias_accel_1->estimate() - vertex_bias_accel_0->estimate();
}

void EdgeAccelBiasRW::linearizeOplus() {
    _jacobianOplusXi = -Mat3d::Identity();
    _jacobianOplusXj = Mat3d::Identity();
}

Mat6d EdgeAccelBiasRW::GetPosteriorInformation() {
    linearizeOplus();

    Eigen::Matrix<double, 3, 6> jacobian = Eigen::Matrix<double, 3, 6>::Zero();
    jacobian.block<3, 3>(0, 0) = _jacobianOplusXi;
    jacobian.block<3, 3>(0, 3) = _jacobianOplusXj;
    return jacobian.transpose() * information() * jacobian;
}
