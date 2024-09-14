//
// Created by Zhang Zhimeng on 22-9-27.
//

#include "common/motion_interpolator.h"

#include <gtest/gtest.h>

TEST(TestMotionInterpolator, TestNormalInput) {
    // test Quaternion interpolate
    Eigen::AngleAxisd yaw_0(1 / 180.0 * M_PI, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yaw_1(3 / 180.0 * M_PI, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q_yaw_0(yaw_0);
    Eigen::Quaterniond q_yaw_1(yaw_1);

    Eigen::Quaterniond q;
    q = MotionInterpolator::InterpolateQuaternionLerp(q_yaw_0, q_yaw_1, 0.5);
    EXPECT_DOUBLE_EQ(Eigen::AngleAxisd(q).angle(), 2 / 180.0 * M_PI);

    q = MotionInterpolator::InterpolateQuaternionLerp(q_yaw_0, q_yaw_1, 0.0);
    EXPECT_DOUBLE_EQ(Eigen::AngleAxisd(q).angle(), 1 / 180.0 * M_PI);

    q = MotionInterpolator::InterpolateQuaternionLerp(q_yaw_0, q_yaw_1, 1.0);
    EXPECT_DOUBLE_EQ(Eigen::AngleAxisd(q).angle(), 3 / 180.0 * M_PI);

    Eigen::AngleAxisd yaw_2(2 / 180.0 * M_PI, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yaw_3(6 / 180.0 * M_PI, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q_yaw_2(yaw_2);
    Eigen::Quaterniond q_yaw_3(yaw_3);
    q = MotionInterpolator::InterpolateQuaternionLerp(q_yaw_2, q_yaw_3, 2.0, 4.0, 3.0);
    EXPECT_DOUBLE_EQ(Eigen::AngleAxisd(q).angle(), 4 / 180.0 * M_PI);

    q = MotionInterpolator::InterpolateQuaternionLerp(q_yaw_2, q_yaw_3, 2.0, 4.0, 2.0);
    EXPECT_DOUBLE_EQ(Eigen::AngleAxisd(q).angle(), 2 / 180.0 * M_PI);

    q = MotionInterpolator::InterpolateQuaternionLerp(q_yaw_2, q_yaw_3, 2.0, 4.0, 4.0);
    EXPECT_DOUBLE_EQ(Eigen::AngleAxisd(q).angle(), 6 / 180.0 * M_PI);

    q = MotionInterpolator::InterpolateQuaternionSlerp(q_yaw_2, q_yaw_3, 2.0, 4.0, 3.0);
    EXPECT_DOUBLE_EQ(Eigen::AngleAxisd(q).angle(), 4 / 180.0 * M_PI);

    q = MotionInterpolator::InterpolateQuaternionSlerp(q_yaw_2, q_yaw_3, 2.0, 4.0, 2.0);
    EXPECT_DOUBLE_EQ(Eigen::AngleAxisd(q).angle(), 2 / 180.0 * M_PI);

    q = MotionInterpolator::InterpolateQuaternionSlerp(q_yaw_2, q_yaw_3, 2.0, 4.0, 4.0);
    EXPECT_DOUBLE_EQ(Eigen::AngleAxisd(q).angle(), 6 / 180.0 * M_PI);

    q = MotionInterpolator::InterpolateQuaternionSlerp(q_yaw_2, q_yaw_3, 0.5);
    EXPECT_DOUBLE_EQ(Eigen::AngleAxisd(q).angle(), 4 / 180.0 * M_PI);

    EXPECT_TRUE(q_yaw_2.slerp(0.5, q_yaw_3).coeffs() == q.coeffs());

    q = MotionInterpolator::InterpolateQuaternionSlerp(q_yaw_2, q_yaw_3, 0.0);
    EXPECT_DOUBLE_EQ(Eigen::AngleAxisd(q).angle(), 2 / 180.0 * M_PI);

    q = MotionInterpolator::InterpolateQuaternionSlerp(q_yaw_2, q_yaw_3, 1.0);
    EXPECT_DOUBLE_EQ(Eigen::AngleAxisd(q).angle(), 6 / 180.0 * M_PI);

    // test vector interpolate
    Eigen::Vector3d v_0(0, 1, 2);
    Eigen::Vector3d v_1(2, 3, 5);

    Eigen::Vector3d v;
    v = MotionInterpolator::InterpolateVectorLerp(v_0, v_1, 0.5);
    EXPECT_TRUE(v == Eigen::Vector3d(1, 2, 3.5));

    v = MotionInterpolator::InterpolateVectorLerp(v_0, v_1, 0.0);
    EXPECT_TRUE(v == v_0);

    v = MotionInterpolator::InterpolateVectorLerp(v_0, v_1, 1.0);
    EXPECT_TRUE(v == v_1);

    v = MotionInterpolator::InterpolateVectorLerp(v_0, v_1, 2, 8, 5);
    EXPECT_TRUE(v == Eigen::Vector3d(1, 2, 3.5));

    Eigen::Vector3d v_2(-1, 1, 0);
    Eigen::Vector3d v_3(1, 1, 0);

    v = MotionInterpolator::InterpolateVectorSlerp(v_2, v_3, 0.5);
    EXPECT_TRUE(v.cast<float>() == Eigen::Vector3d(0, std::sqrt(2), 0).cast<float>());

    v = MotionInterpolator::InterpolateVectorSlerp(v_2, v_3, 1.0, 7.0, 4.0);
    EXPECT_TRUE(v.cast<float>() == Eigen::Vector3d(0, std::sqrt(2), 0).cast<float>());
}