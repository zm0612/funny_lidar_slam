//
// Created by Zhang Zhimeng on 24-1-13.
//

#include "optimization/g2o/velocity_edge.h"

void EdgeVelocity::computeError() {
    const auto *velocity_vertex = dynamic_cast<const VertexVelocity *>(_vertices.at(0));
    _error = measurement() - velocity_vertex->estimate();
}

void EdgeVelocity::linearizeOplus() {
    _jacobianOplusXi = -Mat3d::Identity();
}

Mat3d EdgeVelocity::GetPosteriorInformation() {
    linearizeOplus();

    const Mat3d info = information();
    const Mat3d &jacobian = _jacobianOplusXi;
    return jacobian.transpose() * info * jacobian;
}