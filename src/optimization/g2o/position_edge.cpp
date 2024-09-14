//
// Created by Zhang Zhimeng on 24-1-10.
//

#include "optimization/g2o/position_edge.h"

void EdgePosition::computeError() {
    const auto *position_vertex = dynamic_cast<const VertexPosition *>(_vertices.at(0));
    _error = measurement() - position_vertex->estimate();
}

void EdgePosition::linearizeOplus() {
    _jacobianOplusXi = -Mat3d::Identity();
}

Mat3d EdgePosition::GetPosteriorInformation() {
    linearizeOplus();

    const MatXd info = information();
    const Mat3d &jacobian = _jacobianOplusXi;

    return jacobian.transpose() * info * jacobian;
}