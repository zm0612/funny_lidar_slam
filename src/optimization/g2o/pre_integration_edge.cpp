//
// Created by Zhang Zhimeng on 23-10-21.
//

#include "optimization/g2o/pre_integration_edge.h"
#include "optimization/g2o/vertex_type.h"

#include <glog/logging.h>

EdgePreIntegration::EdgePreIntegration(std::shared_ptr<const PreIntegration> pre_integration_ptr)
        : pre_integration_ptr_(std::move(pre_integration_ptr)) {
    CHECK_NOTNULL(pre_integration_ptr_);
    resize(8);

    setInformation(pre_integration_ptr_->Covariance().inverse());
}

void EdgePreIntegration::linearizeOplus() {
    // last state
    const auto *R_i_vertex = dynamic_cast<const VertexRotation *>(_vertices[0]);
    const auto *V_i_vertex = dynamic_cast<const VertexVelocity *>(_vertices[1]);
    const auto *P_i_vertex = dynamic_cast<const VertexPosition *>(_vertices[2]);

    // last gyro bias
    const auto *bg_vertex = dynamic_cast<const VertexBiasGyro *>(_vertices[3]);

    // current state
    const auto *R_j_vertex = dynamic_cast<const VertexRotation *>(_vertices[5]);
    const auto *V_j_vertex = dynamic_cast<const VertexVelocity *>(_vertices[6]);
    const auto *P_j_vertex = dynamic_cast<const VertexPosition *>(_vertices[7]);

    // state
    const Mat3d &R_i = R_i_vertex->estimate();
    const Mat3d &R_j = R_j_vertex->estimate();
    const Vec3d &V_i = V_i_vertex->estimate();
    const Vec3d &V_j = V_j_vertex->estimate();
    const Vec3d &P_i = P_i_vertex->estimate();
    const Vec3d &P_j = P_j_vertex->estimate();
    const Vec3d delta_bg = bg_vertex->estimate() - pre_integration_ptr_->GetGyroBias();

    // imu pre-integrated time
    const double delta_t = pre_integration_ptr_->GetTotalIntegrationTime();

    // earth gravity
    const Vec3d &gravity = pre_integration_ptr_->GetGravity();

    // derivative of state with respect to bias
    const Mat3d &dR_dbg = pre_integration_ptr_->Get_dR_dbg();
    const Mat3d &dV_dbg = pre_integration_ptr_->Get_dV_dbg();
    const Mat3d &dV_dba = pre_integration_ptr_->Get_dV_dba();
    const Mat3d &dP_dbg = pre_integration_ptr_->Get_dP_dbg();
    const Mat3d &dP_dba = pre_integration_ptr_->Get_dP_dba();

    // imu pre-integrated difference
    const Mat3d &dR_imu = pre_integration_ptr_->DeltaR();

    const Vec3d R_error = SO3Log((dR_imu * SO3Exp(dR_dbg * delta_bg)).transpose() * R_i.transpose() * R_j);

    // 顶点排列顺序如下：
    // [R_i, V_i, P_i, R_j, V_j, P_j, delta_bg, delta_ba]
    // 分别求解误差函数函数以上8个状态量(顶点)的雅克比矩阵
    // 误差函数[delta R, delta V, delta P]^T是一个9x1的列向量
    // 而每个状态量都是3x1的列向量，所以误差对每个顶点的雅克比都是9x3的矩阵
    // 每个雅克比矩阵，0-2行是旋转关于该状态的导数，3-5是速度关于该状态的导数，6-8是位移关于该状态的导数

    // 以下所有公式的求导，参考《预积分推导总结》的第7节

    // derivative of error with respect to R_i
    _jacobianOplus[0].setZero();
    _jacobianOplus[0].block<3, 3>(0, 0) = -So3JacobianRight(R_error).inverse() * R_j.transpose() * R_i;
    _jacobianOplus[0].block<3, 3>(3, 0) = SO3Hat(R_i.transpose() * (V_j - V_i - gravity * delta_t));
    _jacobianOplus[0].block<3, 3>(6, 0) = SO3Hat(R_i.transpose()
                                                 * (P_j - P_i - V_i * delta_t - 0.5 * gravity * delta_t * delta_t));

    // derivative of error with respect to V_i
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3, 3>(0, 0) = Mat3d::Zero();
    _jacobianOplus[1].block<3, 3>(3, 0) = -R_i.transpose();
    _jacobianOplus[1].block<3, 3>(6, 0) = -R_i.transpose() * delta_t;

    // derivative of error with respect to P_i
    _jacobianOplus[2].setZero();
    _jacobianOplus[2].block<3, 3>(0, 0) = Mat3d::Zero();
    _jacobianOplus[2].block<3, 3>(3, 0) = Mat3d::Zero();
    _jacobianOplus[2].block<3, 3>(6, 0) = -R_i.transpose();

    // derivative of error with respect to bg
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3, 3>(0, 0) = -So3JacobianRight(R_error).inverse() * SO3Exp(R_error).transpose()
                                          * So3JacobianRight(dR_dbg * delta_bg) * dR_dbg;
    _jacobianOplus[3].block<3, 3>(3, 0) = -dV_dbg;
    _jacobianOplus[3].block<3, 3>(6, 0) = -dP_dbg;

    // derivative of error with respect to ba
    _jacobianOplus[4].setZero();
    _jacobianOplus[4].block<3, 3>(0, 0) = Mat3d::Zero();
    _jacobianOplus[4].block<3, 3>(3, 0) = -dV_dba;
    _jacobianOplus[4].block<3, 3>(6, 0) = -dP_dba;

    // derivative of error with respect to R_j
    _jacobianOplus[5].setZero();
    _jacobianOplus[5].block<3, 3>(0, 0) = So3JacobianRight(R_error).inverse();
    _jacobianOplus[5].block<3, 3>(3, 0) = Mat3d::Zero();
    _jacobianOplus[5].block<3, 3>(6, 0) = Mat3d::Zero();

    // derivative of error with respect to V_j
    _jacobianOplus[6].setZero();
    _jacobianOplus[6].block<3, 3>(0, 0) = Mat3d::Zero();
    _jacobianOplus[6].block<3, 3>(3, 0) = R_i.transpose();
    _jacobianOplus[6].block<3, 3>(6, 0) = Mat3d::Zero();

    // derivative of error with respect to P_j
    _jacobianOplus[7].setZero();
    _jacobianOplus[7].block<3, 3>(0, 0) = Mat3d::Zero();
    _jacobianOplus[7].block<3, 3>(3, 0) = Mat3d::Zero();
    _jacobianOplus[7].block<3, 3>(6, 0) = R_i.transpose();

}

void EdgePreIntegration::computeError() {
    // last state
    const auto *R_i_vertex = dynamic_cast<const VertexRotation *>(_vertices[0]);
    const auto *V_i_vertex = dynamic_cast<const VertexVelocity *>(_vertices[1]);
    const auto *P_i_vertex = dynamic_cast<const VertexPosition *>(_vertices[2]);

    // last bias
    const auto *bg_vertex = dynamic_cast<const VertexBiasGyro *>(_vertices[3]);
    const auto *ba_vertex = dynamic_cast<const VertexBiasAccel *>(_vertices[4]);

    // current state
    const auto *R_j_vertex = dynamic_cast<const VertexRotation *>(_vertices[5]);
    const auto *V_j_vertex = dynamic_cast<const VertexVelocity *>(_vertices[6]);
    const auto *P_j_vertex = dynamic_cast<const VertexPosition *>(_vertices[7]);

    // state
    const Mat3d &R_i = R_i_vertex->estimate();
    const Mat3d &R_j = R_j_vertex->estimate();
    const Vec3d &V_i = V_i_vertex->estimate();
    const Vec3d &V_j = V_j_vertex->estimate();
    const Vec3d &P_i = P_i_vertex->estimate();
    const Vec3d &P_j = P_j_vertex->estimate();
    const Vec3d delta_bg = bg_vertex->estimate() - pre_integration_ptr_->GetGyroBias();
    const Vec3d delta_ba = ba_vertex->estimate() - pre_integration_ptr_->GetAccBias();

    // earth gravity
    const Vec3d gravity = pre_integration_ptr_->GetGravity();

    // imu pre-integrated time
    const double delta_t = pre_integration_ptr_->GetTotalIntegrationTime();

    // imu pre-integrated difference
    const Mat3d &dR_imu = pre_integration_ptr_->DeltaR();
    const Vec3d &dV_imu = pre_integration_ptr_->DeltaV();
    const Vec3d &dP_imu = pre_integration_ptr_->DeltaP();

    // derivative of state with respect to bias
    const Mat3d &dR_dbg = pre_integration_ptr_->Get_dR_dbg();
    const Mat3d &dV_dbg = pre_integration_ptr_->Get_dV_dbg();
    const Mat3d &dV_dba = pre_integration_ptr_->Get_dV_dba();
    const Mat3d &dP_dbg = pre_integration_ptr_->Get_dP_dbg();
    const Mat3d &dP_dba = pre_integration_ptr_->Get_dP_dba();

    // rotation error. formula (23)
    _error.block<3, 1>(0, 0) = SO3Log((dR_imu * SO3Exp(dR_dbg * delta_bg)).transpose() * R_i.transpose() * R_j);

    // velocity error. formula (24)
    _error.block<3, 1>(3, 0) = R_i.transpose() * (V_j - V_i - gravity * delta_t)
                               - (dV_imu + dV_dbg * delta_bg + dV_dba * delta_ba);

    // position error. formula (25)
    _error.block<3, 1>(6, 0) = R_i.transpose() * (P_j - P_i - V_i * delta_t - 0.5 * gravity * delta_t * delta_t)
                               - (dP_imu + dP_dbg * delta_bg + dP_dba * delta_ba);
}

Mat24d EdgePreIntegration::GetPosteriorInformation() {
    linearizeOplus();

    const Mat9d info = information();

    Eigen::Matrix<double, 9, 24> jacobian;
    jacobian.setZero();
    jacobian.block<9, 3>(0, 0) = _jacobianOplus[0]; // delta_e / delta_R_i
    jacobian.block<9, 3>(0, 3) = _jacobianOplus[1]; // delta_e / delta_V_i
    jacobian.block<9, 3>(0, 6) = _jacobianOplus[2]; // delta_e / delta_P_i
    jacobian.block<9, 3>(0, 9) = _jacobianOplus[3]; // delta_e / delta_bg_i
    jacobian.block<9, 3>(0, 12) = _jacobianOplus[4]; // delta_e / delta_ba_i
    jacobian.block<9, 3>(0, 15) = _jacobianOplus[5]; // delta_e / delta_R_j
    jacobian.block<9, 3>(0, 18) = _jacobianOplus[6]; // delta_e / delta_V_j
    jacobian.block<9, 3>(0, 21) = _jacobianOplus[7]; // delta_e / delta_P_j

    return jacobian.transpose() * info * jacobian;
}