//
// Created by Zhang Zhimeng on 23-11-15.
//
#include "optimization/g2o/prior_nav_state_edge.h"

EdgePriorNavState::EdgePriorNavState(const NavStateData &nav_state, const Mat15d &info) {
    nav_state_ = nav_state;

    resize(5);
    setInformation(info);
}

void EdgePriorNavState::linearizeOplus() {
    const auto *rotation_vertex = dynamic_cast<const VertexRotation *>(_vertices.at(0));

    // error = measure - estimate
    // error dim: 15
    // vertex: R, V, P, delta_bg, delta_ba. all dim 3 x 1
    // So, Jacobian dim: 15 x 3

    const Vec3d err_rotation = SO3Log(nav_state_.R_.transpose() * rotation_vertex->estimate());

    // derivative of error with respect to R
    _jacobianOplus.at(0).setZero();
    _jacobianOplus.at(0).block<3, 3>(0, 0) = So3JacobianRight(err_rotation).inverse();

    // derivative of error with respect to V
    _jacobianOplus.at(1).setZero();
    _jacobianOplus.at(1).block<3, 3>(3, 0) = -Mat3d::Identity();

    // derivative of error with respect to P
    _jacobianOplus.at(2).setZero();
    _jacobianOplus.at(2).block<3, 3>(6, 0) = -Mat3d::Identity();

    // derivative of error with respect to bg
    _jacobianOplus.at(3).setZero();
    _jacobianOplus.at(3).block<3, 3>(9, 0) = -Mat3d::Identity();

    // derivative of error with respect to ba
    _jacobianOplus.at(4).setZero();
    _jacobianOplus.at(4).block<3, 3>(12, 0) = -Mat3d::Identity();
}

void EdgePriorNavState::computeError() {
    const auto *rotation_vertex = dynamic_cast<const VertexRotation *>(_vertices.at(0));
    const auto *velocity_vertex = dynamic_cast<const VertexVelocity *>(_vertices.at(1));
    const auto *position_vertex = dynamic_cast<const VertexPosition *>(_vertices.at(2));
    const auto *bg_vertex = dynamic_cast<const VertexBiasGyro *>(_vertices.at(3));
    const auto *ba_vertex = dynamic_cast<const VertexBiasAccel *>(_vertices.at(4));

    // error = measure - estimate
    _error.block<3, 1>(0, 0) = SO3Log(nav_state_.R_.transpose() * rotation_vertex->estimate());
    _error.block<3, 1>(3, 0) = nav_state_.V_ - velocity_vertex->estimate();
    _error.block<3, 1>(6, 0) = nav_state_.P_ - position_vertex->estimate();
    _error.block<3, 1>(9, 0) = nav_state_.bg_ - bg_vertex->estimate();
    _error.block<3, 1>(12, 0) = nav_state_.ba_ - ba_vertex->estimate();
}

Mat15d EdgePriorNavState::GetPosteriorInformation() {
    linearizeOplus();

    const MatXd info = information();
    Mat15d jacobian;
    jacobian.setZero();
    jacobian.block<15, 3>(0, 0) = _jacobianOplus[0];
    jacobian.block<15, 3>(0, 3) = _jacobianOplus[1];
    jacobian.block<15, 3>(0, 6) = _jacobianOplus[2];
    jacobian.block<15, 3>(0, 9) = _jacobianOplus[3];
    jacobian.block<15, 3>(0, 12) = _jacobianOplus[4];

    return jacobian.transpose() * info * jacobian;
}