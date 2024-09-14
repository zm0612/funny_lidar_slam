//
// Created by Zhang Zhimeng on 22-8-31.
//

#ifndef FUNNY_LIDAR_SLAM_MOTION_INTERPOLATOR_H
#define FUNNY_LIDAR_SLAM_MOTION_INTERPOLATOR_H

#include <Eigen/Dense>
#include <glog/logging.h>

/*!
 * 该类用来对旋转和向量进行插值，包含四元数和向量的线性插值和球形插值
 * 提示：球形插值精度更高。
 *      但是多数情况下的平移运动和旋转运动使用线性插值就够用了
 * reference: https://zhuanlan.zhihu.com/p/87418561
 */
class MotionInterpolator {
public:
    /*!
     * Quaternion Normalized Linear Interpolation
     * @param q_0 Quaternion start
     * @param q_1 Quaternion end
     * @param t ratio
     * @return
     */
    template<class Derived>
    static Eigen::Quaterniond InterpolateQuaternionLerp(const Eigen::QuaternionBase<Derived> &q_0,
                                                        const Eigen::QuaternionBase<Derived> &q_1,
                                                        typename Derived::Scalar t) {
        CHECK_LE(t, 1.0);
        CHECK_GE(t, 0.0);

        Derived q(q_0.coeffs() * ((typename Derived::Scalar(1.0)) - t) + q_1.coeffs() * t);
        return q.normalized();
    }

    /*!
     * Quaternion Normalized Linear Interpolation
     * @param q_0 Quaternion start
     * @param q_1 Quaternion end
     * @param t_0 start time
     * @param t_1 end time
     * @param t interpolated time point
     * @return
     */
    template<class Derived>
    static Eigen::Quaterniond InterpolateQuaternionLerp(const Eigen::QuaternionBase<Derived> &q_0,
                                                        const Eigen::QuaternionBase<Derived> &q_1,
                                                        uint64_t t_0,
                                                        uint64_t t_1,
                                                        uint64_t t) {
        CHECK_GT(t_1, t_0);
        CHECK_GE(t, t_0);
        CHECK_LE(t, t_1);

        typename Derived::Scalar temp_t = static_cast<typename Derived::Scalar>(t - t_0)
                                          / static_cast<typename Derived::Scalar>(t_1 - t_0);
        Derived q(q_0.coeffs() * ((typename Derived::Scalar(1.0)) - temp_t) + q_1.coeffs() * temp_t);
        return q.normalized();
    }

    /*!
     * Quaternion Spherical Linear Interpolation
     * @tparam Derived
     * @param q_0 Quaternion start
     * @param q_1 Quaternion end
     * @param t ratio
     * @return
     */
    template<class Derived>
    static Derived InterpolateQuaternionSlerp(const Eigen::QuaternionBase<Derived> &q_0,
                                              const Eigen::QuaternionBase<Derived> &q_1,
                                              typename Derived::Scalar t) {
        CHECK_LE(t, 1.0);
        CHECK_GE(t, 0.0);

        typename Derived::Scalar one =
                typename Derived::Scalar(1) - std::numeric_limits<typename Derived::Scalar>::epsilon();
        typename Derived::Scalar d = q_0.template dot(q_1);
        typename Derived::Scalar abs_d = std::abs(d);

        typename Derived::Scalar scale_0;
        typename Derived::Scalar scale_1;

        if (abs_d >= one) {
            scale_0 = typename Derived::Scalar(1) - t;
            scale_1 = t;
        } else {
            typename Derived::Scalar theta = std::acos(abs_d);
            typename Derived::Scalar sin_theta = std::sin(theta);

            scale_0 = std::sin((typename Derived::Scalar(1) - t) * theta) / sin_theta;
            scale_1 = std::sin(t * theta) / sin_theta;
        }

        if (d < typename Derived::Scalar(0)) {
            scale_1 = -scale_1;
        }

        Derived q;
        q = scale_0 * q_0.coeffs() + scale_1 * q_1.coeffs();

        return q;
    }

    /*!
     * @tparam Derived
     * @param q_0 Quaternion start
     * @param q_1 Quaternion end
     * @param t_0 start time
     * @param t_1 end time
     * @param t interpolated time point
     * @return
     */
    template<typename Derived>
    static Derived InterpolateQuaternionSlerp(const Eigen::QuaternionBase<Derived> &q_0,
                                              const Eigen::QuaternionBase<Derived> &q_1,
                                              uint64_t t_0,
                                              uint64_t t_1,
                                              uint64_t t) {
        CHECK_GT(t_1, t_0);
        CHECK_GE(t, t_0);
        CHECK_LE(t, t_1);
        typename Derived::Scalar temp_t = static_cast<typename Derived::Scalar>(t - t_0)
                                          / static_cast<typename Derived::Scalar>(t_1 - t_0);

        typename Derived::Scalar one =
                typename Derived::Scalar(1) - std::numeric_limits<typename Derived::Scalar>::epsilon();
        typename Derived::Scalar d = q_0.template dot(q_1);
        typename Derived::Scalar abs_d = std::abs(d);

        typename Derived::Scalar scale_0;
        typename Derived::Scalar scale_1;

        if (abs_d >= one) {
            scale_0 = typename Derived::Scalar(1) - temp_t;
            scale_1 = temp_t;
        } else {
            typename Derived::Scalar theta = std::acos(abs_d);
            typename Derived::Scalar sin_theta = std::sin(theta);

            scale_0 = std::sin((typename Derived::Scalar(1) - temp_t) * theta) / sin_theta;
            scale_1 = std::sin(temp_t * theta) / sin_theta;
        }

        if (d < typename Derived::Scalar(0)) {
            scale_1 = -scale_1;
        }

        Derived q;
        q = scale_0 * q_0.coeffs() + scale_1 * q_1.coeffs();

        return q;
    }


    /*!
     * Vector Normalized Linear Interpolation
     * @tparam Derived
     * @param v_0 vector star
     * @param v_1 vector end
     * @param t ration
     * @return
     */
    template<class Derived>
    static Derived InterpolateVectorLerp(const Eigen::MatrixBase<Derived> &v_0,
                                         const Eigen::MatrixBase<Derived> &v_1,
                                         typename Derived::Scalar t) {
        CHECK_LE(t, 1.0);
        CHECK_GE(t, 0.0);

        return ((typename Derived::Scalar(1.0)) - t) * v_0 + t * v_1;
    }

    /*!
     * Vector Normalized Linear Interpolation
     * @tparam Derived
     * @param v_0 vector start
     * @param v_1 vector end
     * @param t_0 time start
     * @param t_1 time end
     * @param t interpolated time point
     * @return
     */
    template<class Derived>
    static Derived InterpolateVectorLerp(const Eigen::MatrixBase<Derived> &v_0,
                                         const Eigen::MatrixBase<Derived> &v_1,
                                         uint64_t t_0,
                                         uint64_t t_1,
                                         uint64_t t
    ) {
        CHECK_GT(t_1, t_0);
        CHECK_GE(t, t_0);
        CHECK_LE(t, t_1);

        typename Derived::Scalar temp_t = static_cast<typename Derived::Scalar>(t - t_0)
                                          / static_cast<typename Derived::Scalar>(t_1 - t_0);

        return ((typename Derived::Scalar(1.0)) - temp_t) * v_0 + temp_t * v_1;
    }

    /*!
     * Vector Spherical Linear Interpolation
     * @tparam Derived
     * @param v_0 vector start
     * @param v_1 vector end
     * @param t ratio
     * @return
     */
    template<class Derived>
    static Derived InterpolateVectorSlerp(const Eigen::MatrixBase<Derived> &v_0,
                                          const Eigen::MatrixBase<Derived> &v_1,
                                          typename Derived::Scalar t) {
        CHECK_LE(t, 1.0);
        CHECK_GE(t, 0.0);

        typename Derived::Scalar theta = std::acos(v_0.dot(v_1));

        Derived v;

        v = std::sin(((typename Derived::Scalar(1.0)) - t) * theta) / std::sin(theta) * v_0
            + std::sin(t * theta) / std::sin(theta) * v_1;

        return v;
    }

    /*!
     * Vector Spherical Linear Interpolation
     * @tparam Derived
     * @param v_0 vector start
     * @param v_1 vector end
     * @param t_0 time start
     * @param t_1 time end
     * @param t interpolated time point
     * @return
     */
    template<class Derived>
    static Derived InterpolateVectorSlerp(const Eigen::MatrixBase<Derived> &v_0,
                                          const Eigen::MatrixBase<Derived> &v_1,
                                          uint64_t t_0,
                                          uint64_t t_1,
                                          uint64_t t) {
        CHECK_GT(t_1, t_0);
        CHECK_GE(t, t_0);
        CHECK_LE(t, t_1);

        typename Derived::Scalar temp_t = static_cast<typename Derived::Scalar>(t - t_0)
                                          / static_cast<typename Derived::Scalar>(t_1 - t_0);

        typename Derived::Scalar theta = std::acos(v_0.dot(v_1));

        Derived v;
        v = std::sin(((typename Derived::Scalar(1.0)) - temp_t) * theta) / std::sin(theta) * v_0
            + std::sin(temp_t * theta) / std::sin(theta) * v_1;

        return v;
    }

    MotionInterpolator() = delete;

    MotionInterpolator(MotionInterpolator &) = delete;

    ~MotionInterpolator() = delete;
};

#endif //FUNNY_LIDAR_SLAM_MOTION_INTERPOLATOR_H
