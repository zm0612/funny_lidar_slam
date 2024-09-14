//
// Created by Zhang Zhimeng on 22-4-26.
//

#ifndef FUNNY_LIDAR_SLAM_MATH_FUNCTION_H
#define FUNNY_LIDAR_SLAM_MATH_FUNCTION_H

#include "common/data_type.h"

#include <cmath>
#include <Eigen/Dense>
#include <numeric>

#define kMicroseconds2Seconds (1.0 / 1.0e6)
#define kRadian2Degree (180.0 / M_PI)
#define kDegree2Radian (M_PI / 180.0)

template<typename T>
inline T Degree2Radian(const T &angle) {
    return angle / 180.0 * M_PI;
}

template<typename T>
inline T Radian2Angle(const T &radian) {
    return radian * 180.0 / M_PI;
}

/*!
 * 转换角度到 [-Pi, Pi)
 * @tparam T
 * @param x
 * @return
 */
template<typename T>
inline T Mod2Pi(const T &x) {
    T v = x;

    if (std::abs(x) > static_cast<T>(2.0) * M_PI) {
        v = std::fmod(x, M_PI * static_cast<T>(2.0));
    }

    if (v < -M_PI) {
        v += static_cast<T>(2.0) * M_PI;
    } else if (v >= M_PI) {
        v -= static_cast<T>(2.0) * M_PI;
    }

    return v;
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SO3Hat(const Eigen::MatrixBase<Derived> &v) {
    eigen_assert(v.size() == 3u);

    Eigen::Matrix<typename Derived::Scalar, 3, 3> skew_mat;
    skew_mat.setZero();
    skew_mat(0, 1) = -v(2);
    skew_mat(0, 2) = +v(1);
    skew_mat(1, 2) = -v(0);
    skew_mat(1, 0) = +v(2);
    skew_mat(2, 0) = -v(1);
    skew_mat(2, 1) = +v(0);
    return skew_mat;
}

/*!
 * so3 exponential mapping to SO3
 * v = theta * v_normalized
 * @tparam Derived
 * @param v
 * @return
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SO3Exp(const Eigen::MatrixBase<Derived> &v) {
    eigen_assert(v.size() == 3u);

    Eigen::Matrix<typename Derived::Scalar, 3, 3> R = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity();
    typename Derived::Scalar theta = v.norm();

    if (theta > std::numeric_limits<typename Derived::Scalar>::epsilon()) {
        Eigen::Matrix<typename Derived::Scalar, 3, 1> v_normalized = v.normalized();
        R = std::cos(theta) * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity()
            + (typename Derived::Scalar(1.0) - std::cos(theta)) * v_normalized *
              v_normalized.transpose() + std::sin(theta) * SO3Hat(v_normalized);
        return R;
    } else {
        return R;
    }
}

/*!
 * SE3 Exponential Mapping
 * @tparam Derived
 * @param v  [translation, rotation]
 * @return
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> SE3Exp(const Eigen::MatrixBase<Derived> &v) {
    eigen_assert(v.size() == 6u);
    using std::cos;
    using std::sin;
    using Mat3 = Eigen::Matrix<typename Derived::Scalar, 3, 3>;
    using Vec3 = Eigen::Matrix<typename Derived::Scalar, 3, 1>;
    using Scalar = typename Derived::Scalar;

    const Vec3 omega = v.template tail<3>();
    Scalar theta = omega.norm();
    Mat3 R = SO3Exp(omega);
    Mat3 omega_hat = SO3Hat(omega);
    Mat3 omega_hat_sq = omega_hat * omega_hat;
    Mat3 J;

    if (theta < std::numeric_limits<typename Derived::Scalar>::epsilon()) {
        J = R;
    } else {
        Scalar theta_sq = theta * theta;
        J = (Mat3::Identity() +
             (Scalar(1.0) - cos(theta)) / (theta_sq) * omega_hat +
             (theta - sin(theta)) / (theta_sq * theta) * omega_hat_sq);
    }

    Eigen::Matrix<typename Derived::Scalar, 4, 4> T;
    T.setIdentity();
    T.template block<3, 3>(0, 0) = R;
    T.template block<3, 1>(0, 3) = J * v.template head<3>();

    return T;
}

/*!
 * get roll, pith, yaw angle from rotation matrix
 * note: 该函数与Eigen中的euler()函数有一些区别，该函数的结果不会出现+Pi或者-Pi的现象
 *       此函数是定轴转动，使用的是左乘 R = Rz * Ry * Rx，求解出来的欧拉角rpy分别对应Rx, Ry, Rz
 * @tparam Derived
 * @param R
 * @return
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> RotationMatrixToRPY(const Eigen::MatrixBase<Derived> &R) {
    eigen_assert(R.rows() == 3u);
    eigen_assert(R.cols() == 3u);

    typename Derived::Scalar roll, pitch, yaw;
    roll = std::atan2(R(2, 1), R(2, 2));
    pitch = std::asin(-R(2, 0));
    yaw = std::atan2(R(1, 0), R(0, 0));

    return {roll, pitch, yaw};
}

/*!
 * Fast atan2.
 * Can guarantee angular accuracy of two decimal places
 * @tparam Type
 * @param y
 * @param x
 * @return
 */
template<typename Type>
inline float FastAtan2(Type y, Type x) {
    constexpr Type atan2_p1 = Type(0.9997878412794807);
    constexpr Type atan2_p3 = Type(-0.3258083974640975);
    constexpr Type atan2_p5 = Type(0.1555786518463281);
    constexpr Type atan2_p7 = Type(-0.04432655554792128);
    Type ax = std::fabs(x), ay = std::fabs(y);
    Type a, c, c2;
    if (ax >= ay) {
        c = ay / (ax + std::numeric_limits<Type>::epsilon());
        c2 = c * c;
        a = (((atan2_p7 * c2 + atan2_p5) * c2 + atan2_p3) * c2 + atan2_p1) * c;
    } else {
        c = ax / (ay + std::numeric_limits<Type>::epsilon());
        c2 = c * c;
        a = Type(M_PI_2) - (((atan2_p7 * c2 + atan2_p5) * c2 + atan2_p3) * c2 + atan2_p1) * c;
    }
    if (x < 0)
        a = Type(M_PI) - a;
    if (y < 0)
        a = Type(2 * M_PI) - a;

    if (a > Type(M_PI)) {
        a -= Type(2 * M_PI);
    }

    return a;
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> So3JacobianLeft(const Eigen::MatrixBase<Derived> &v) {
    eigen_assert(v.size() == 3u);

    const typename Derived::Scalar phi = v.norm();
    const typename Eigen::Matrix<typename Derived::Scalar, 3, 1> v_normlized = v.normalized();

    if (phi <= std::numeric_limits<typename Derived::Scalar>::epsilon()) {
        return Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity();
    } else {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> Jl;
        Jl = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() * std::sin(phi) / phi +
             (typename Derived::Scalar(1.0) - std::sin(phi) / phi) * v_normlized * v_normlized.transpose()
             + (typename Derived::Scalar(1.0) - std::cos(phi)) / phi * SO3Hat(v_normlized);

        return Jl;
    }
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> So3JacobianRight(const Eigen::MatrixBase<Derived> &v) {
    eigen_assert(v.size() == 3u);

    const typename Derived::Scalar phi = v.norm();
    const typename Eigen::Matrix<typename Derived::Scalar, 3, 1> v_normlized = v.normalized();

    const Eigen::Matrix<typename Derived::Scalar, 3, 3> v_normlized_hat = SO3Hat(v_normlized);

    if (phi < std::numeric_limits<typename Derived::Scalar>::epsilon()) {
        return Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity();
    } else {
        const typename Derived::Scalar s2 = std::sin(phi / typename Derived::Scalar(2.0));
        const typename Derived::Scalar one_minus_cos = typename Derived::Scalar(2.0) * s2 * s2;
        const typename Derived::Scalar one_minus_sin_div_phi = (typename Derived::Scalar(1.0) - std::sin(phi) / phi);

        Eigen::Matrix<typename Derived::Scalar, 3, 3> Jr;
        Jr = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() -
             one_minus_cos / phi * v_normlized_hat
             + one_minus_sin_div_phi * v_normlized_hat * v_normlized_hat;

        return Jr;
    }
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 6, 6> SE3LeftJacobian(
        const Eigen::MatrixBase<Derived> &upsilon_omega) {
    eigen_assert(upsilon_omega.size() == 6);

    using Scalar = typename Derived::Scalar;
    using Mat3 = Eigen::Matrix<typename Derived::Scalar, 3, 3>;
    using Mat6 = Eigen::Matrix<typename Derived::Scalar, 6, 6>;
    using Vec3 = Eigen::Matrix<typename Derived::Scalar, 3, 1>;

    auto JacobianUpperRightBlock = [](Vec3 const &upsilon, Vec3 const &omega) {
        using std::cos;
        using std::sin;
        using std::sqrt;

        const Scalar k1By2(0.5);

        const Scalar theta_sq = omega.squaredNorm();
        const Mat3 Upsilon = SO3Hat(upsilon);

        Mat3 Q;
        if (theta_sq < std::numeric_limits<Scalar>::epsilon() * std::numeric_limits<Scalar>::epsilon()) {
            Q = k1By2 * Upsilon;
        } else {
            const Scalar theta = sqrt(theta_sq);
            const Scalar i_theta = Scalar(1) / theta;
            const Scalar i_theta_sq = i_theta * i_theta;
            const Scalar i_theta_po4 = i_theta_sq * i_theta_sq;
            const Scalar st = sin(theta);
            const Scalar ct = cos(theta);
            const Scalar c1 = i_theta_sq - st * i_theta_sq * i_theta;
            const Scalar c2 = k1By2 * i_theta_sq + ct * i_theta_po4 - i_theta_po4;
            const Scalar c3 = i_theta_po4 + k1By2 * ct * i_theta_po4 -
                              Scalar(1.5) * st * i_theta * i_theta_po4;

            const Mat3 Omega = SO3Hat(omega);
            const Mat3 OmegaUpsilon = Omega * Upsilon;
            const Mat3 OmegaUpsilonOmega = OmegaUpsilon * Omega;
            Q = k1By2 * Upsilon +
                c1 * (OmegaUpsilon + Upsilon * Omega + OmegaUpsilonOmega) -
                c2 * (theta_sq * Upsilon + Scalar(2) * OmegaUpsilonOmega) +
                c3 * (OmegaUpsilonOmega * Omega + Omega * OmegaUpsilonOmega);
        }
        return Q;
    };

    const Vec3 upsilon = upsilon_omega.template head<3>();
    const Vec3 omega = upsilon_omega.template tail<3>();
    const Mat3 J = So3JacobianLeft(omega);
    const Mat3 Q = JacobianUpperRightBlock(upsilon, omega);
    Mat6 U;
    U << J, Q, Mat3::Zero(), J;

    return U;
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 6, 6> SE3RightJacobian(
        const Eigen::MatrixBase<Derived> &upsilon_omega
) {
    eigen_assert(upsilon_omega.size() == 6);
    return SE3LeftJacobian(-upsilon_omega);
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 6, 6> SE3Adj(const Eigen::MatrixBase<Derived> &T) {
    using Mat3 = Eigen::Matrix<typename Derived::Scalar, 3, 3>;
    using Mat6 = Eigen::Matrix<typename Derived::Scalar, 6, 6>;
    const Mat3 R = T.template block<3, 3>(0, 0);

    Mat6 res;
    res.block(0, 0, 3, 3) = R;
    res.block(3, 3, 3, 3) = R;
    res.block(0, 3, 3, 3) = SO3Hat(T.template block<3, 1>(0, 3)) * R;
    res.block(3, 0, 3, 3) = Mat3::Zero();
    return res;
}

/*!
 * refer to:
 * a. paper: Quaternion kinematics for the error-state Kalman filter
 * b. https://thenumb.at/Exponential-Rotations/#the-exponential-and-logarithmic-maps
 * @tparam Derived
 * @param R
 * @return
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> SO3Log(const Eigen::MatrixBase<Derived> &R) {
    eigen_assert(R.rows() == 3);
    eigen_assert(R.cols() == 3);

    Eigen::Quaternion<typename Derived::Scalar> q(R);

    if (q.norm() <= std::numeric_limits<typename Derived::Scalar>::epsilon()) {
        Eigen::Matrix<typename Derived::Scalar, 3, 1> phi_u;
        phi_u = typename Derived::Scalar(2.0) * q.vec() / q.w() *
                (typename Derived::Scalar(1.0) -
                 q.vec().squaredNorm() / (typename Derived::Scalar(3.0) * q.w() * q.w()));

        return phi_u;
    }

    typename Derived::Scalar norm_vec = q.vec().norm();

    typename Derived::Scalar phi = typename Derived::Scalar(2.0) * (
            (q.w() < typename Derived::Scalar(0.0)) ? typename Derived::Scalar(std::atan2(-norm_vec, -q.w()))
                                                    : typename Derived::Scalar(std::atan2(norm_vec, q.w())));

    Eigen::Matrix<typename Derived::Scalar, 3, 1> u;
    u = q.vec().normalized();

    return phi * u;
}

/*!
 *
 * @tparam Derived
 * @param T transform matrix
 * @return [t, r]
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 6, 1> SE3Log(const Eigen::MatrixBase<Derived> &T) {
    eigen_assert(T.rows() == 4);
    eigen_assert(T.cols() == 4);

    using std::abs;
    using std::cos;
    using std::sin;
    using Mat3 = Eigen::Matrix<typename Derived::Scalar, 3, 3>;
    using Vec3 = Eigen::Matrix<typename Derived::Scalar, 3, 1>;
    using Vec6 = Eigen::Matrix<typename Derived::Scalar, 6, 1>;
    using Scalar = typename Derived::Scalar;
    Vec6 epsilon_omega;
    Vec3 omega = SO3Log(T.template block<3, 3>(0, 0));
    Scalar theta = omega.norm();

    epsilon_omega.template tail<3>() = omega;
    Mat3 omega_hat = SO3Hat(epsilon_omega.template tail<3>());

    if (abs(theta) < std::numeric_limits<Scalar>::epsilon()) {
        Mat3 V_inv = Mat3::Identity() - Scalar(0.5) * omega_hat +
                     Scalar(1. / 12.) * (omega_hat * omega_hat);

        epsilon_omega.template head<3>() = V_inv * T.template block<3, 1>(0, 3);
    } else {
        Scalar const half_theta = Scalar(0.5) * theta;

        Mat3 V_inv = (Mat3::Identity() - Scalar(0.5) * omega_hat +
                      (Scalar(1.0) - theta * cos(half_theta) / (Scalar(2.0) * sin(half_theta))) /
                      (theta * theta) * (omega_hat * omega_hat));
        epsilon_omega.template head<3>() = V_inv * T.template block<3, 1>(0, 3);
    }
    return epsilon_omega;
}

/*!
 * Marginalize matrix
 * note: Copy from ORB-SLAM3
 * @param H
 * @param start
 * @param end
 * @return
 */
inline MatXd Marginalize(const MatXd &H, const int &start, const int &end) {
    // Goal
    // a  | ab | ac       a*  | 0 | ac*
    // ba | b  | bc  -->  0   | 0 | 0
    // ca | cb | c        ca* | 0 | c*

    // Size of block before block to marginalize
    const int a = start;
    // Size of block to marginalize
    const int b = end - start + 1;
    // Size of block after block to marginalize
    const int c = static_cast<int>(H.cols()) - (end + 1);

    // Reorder as follows:
    // a  | ab | ac       a  | ac | ab
    // ba | b  | bc  -->  ca | c  | cb
    // ca | cb | c        ba | bc | b

    MatXd Hn = MatXd::Zero(H.rows(), H.cols());
    if (a > 0) {
        Hn.block(0, 0, a, a) = H.block(0, 0, a, a);
        Hn.block(0, a + c, a, b) = H.block(0, a, a, b);
        Hn.block(a + c, 0, b, a) = H.block(a, 0, b, a);
    }
    if (a > 0 && c > 0) {
        Hn.block(0, a, a, c) = H.block(0, a + b, a, c);
        Hn.block(a, 0, c, a) = H.block(a + b, 0, c, a);
    }
    if (c > 0) {
        Hn.block(a, a, c, c) = H.block(a + b, a + b, c, c);
        Hn.block(a, a + c, c, b) = H.block(a + b, a, c, b);
        Hn.block(a + c, a, b, c) = H.block(a, a + b, b, c);
    }
    Hn.block(a + c, a + c, b, b) = H.block(a, a, b, b);

    // Perform marginalization (Schur complement)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hn.block(a + c, a + c, b, b), Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv = svd.singularValues();
    for (int i = 0; i < b; ++i) {
        if (singularValues_inv(i) > 1e-6)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else
            singularValues_inv(i) = 0;
    }

    MatXd invHb = svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose();
    Hn.block(0, 0, a + c, a + c) =
            Hn.block(0, 0, a + c, a + c) - Hn.block(0, a + c, a + c, b) * invHb * Hn.block(a + c, 0, b, a + c);
    Hn.block(a + c, a + c, b, b) = Eigen::MatrixXd::Zero(b, b);
    Hn.block(0, a + c, a + c, b) = Eigen::MatrixXd::Zero(a + c, b);
    Hn.block(a + c, 0, b, a + c) = Eigen::MatrixXd::Zero(b, a + c);

    // Inverse reorder
    // a*  | ac* | 0       a*  | 0 | ac*
    // ca* | c*  | 0  -->  0   | 0 | 0
    // 0   | 0   | 0       ca* | 0 | c*
    MatXd res = MatXd::Zero(H.rows(), H.cols());
    if (a > 0) {
        res.block(0, 0, a, a) = Hn.block(0, 0, a, a);
        res.block(0, a, a, b) = Hn.block(0, a + c, a, b);
        res.block(a, 0, b, a) = Hn.block(a + c, 0, b, a);
    }
    if (a > 0 && c > 0) {
        res.block(0, a + b, a, c) = Hn.block(0, a, a, c);
        res.block(a + b, 0, c, a) = Hn.block(a, 0, c, a);
    }
    if (c > 0) {
        res.block(a + b, a + b, c, c) = Hn.block(a, a, c, c);
        res.block(a + b, a, c, b) = Hn.block(a, a + c, c, b);
        res.block(a, a + b, b, c) = Hn.block(a + c, a, b, c);
    }

    res.block(a, a, b, b) = Hn.block(a + c, a + c, b, b);

    return res;
}

#endif //FUNNY_LIDAR_SLAM_MATH_FUNCTION_H
