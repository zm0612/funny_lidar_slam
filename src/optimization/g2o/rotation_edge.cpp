//
// Created by Zhang Zhimeng on 24-1-14.
//

#include "optimization/g2o/rotation_edge.h"

void EdgeRotation::linearizeOplus() {
    const auto *rotation_vertex = dynamic_cast<const VertexRotation *>(_vertices.at(0));
    const Mat3d &R_measure = measurement();

    // error = measure - estimate
    // error dim: 3
    // vertex: R. all dim 3 x 1
    // So, Jacobian dim: 3 x 3
    const Vec3d err_rotation = SO3Log(R_measure.transpose() * rotation_vertex->estimate());

    // derivative of error with respect to R
    _jacobianOplusXi = So3JacobianRight(err_rotation).inverse();
}

void EdgeRotation::computeError() {
    // error = measure - estimate
    // error dim: 3
    // vertex: R
    // So, Jacobian dim: 3 x 3
    const auto *rotation_vertex = dynamic_cast<const VertexRotation *>(_vertices.at(0));
    const Mat3d &R_measure = measurement();

    // error = measure - estimate
    _error = SO3Log(R_measure.transpose() * rotation_vertex->estimate());
}

Mat3d EdgeRotation::GetPosteriorInformation() {
    linearizeOplus();

    const Mat3d info = information();
    const Mat3d jacobian = _jacobianOplusXi;

    return jacobian.transpose() * info * jacobian;
}

