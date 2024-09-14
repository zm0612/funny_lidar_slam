//
// Created by Zhang Zhimeng on 24-1-17.
//
#include "optimization/g2o/gyro_bias_prior_edge.h"

void EdgeGyroBiasPrior::computeError() {
    const auto* bias_gyro_vertex = dynamic_cast<const VertexBiasGyro*>(_vertices.at(0));
    _error = measurement() - bias_gyro_vertex->estimate();
}

void EdgeGyroBiasPrior::linearizeOplus() {
    _jacobianOplusXi = -Mat3d::Identity();
}

Mat3d EdgeGyroBiasPrior::GetPosteriorInformation() {
    linearizeOplus();

    const MatXd info = information();
    const Mat3d& jacobian = _jacobianOplusXi;

    return jacobian.transpose() * info * jacobian;
}
