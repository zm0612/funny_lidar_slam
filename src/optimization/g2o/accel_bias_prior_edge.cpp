//
// Created by Zhang Zhimeng on 24-1-17.
//
#include "optimization/g2o/accel_bias_prior_edge.h"

void EdgeAccelBiasPrior::computeError() {
    const auto *bias_accel_vertex = dynamic_cast<const VertexBiasAccel *>(_vertices.at(0));
    _error = measurement() - bias_accel_vertex->estimate();
}

void EdgeAccelBiasPrior::linearizeOplus() {
    _jacobianOplusXi = -Mat3d::Identity();
}

Mat3d EdgeAccelBiasPrior::GetPosteriorInformation() {
    linearizeOplus();

    const MatXd info = information();
    const Mat3d &jacobian = _jacobianOplusXi;
    return jacobian.transpose() * info * jacobian;
}
