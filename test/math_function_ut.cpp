//
// Created by Zhang Zhimeng on 23-8-19.
//

#include "common/math_function.h"

#include <gtest/gtest.h>

TEST(HatTest, HandleZeroInput) {
    Eigen::Vector3d test_data(0.0, 0.0, 0.0);
    Eigen::Matrix3d test_result = Eigen::Matrix3d::Zero();

    EXPECT_TRUE(test_result.isApprox(SO3Hat(test_data)));
}

TEST(HatTest, HandleIntInput) {
    Eigen::Vector3i test_data(1, 2, 3);
    Eigen::Matrix3i test_result;
    test_result << 0, -3, 2,
            3, 0, -1,
            -2, 1, 0;

    EXPECT_TRUE(test_result.isApprox(SO3Hat(test_data)));
}

TEST(HatTest, HandleFloatInput) {
    Eigen::Vector3f test_data(1.0f, 2.0f, 3.0f);
    Eigen::Matrix3f test_result;
    test_result << 0.0f, -3.0f, 2.0f,
            3.0f, 0.0f, -1.0f,
            -2.0f, 1.0f, 0.0f;

    EXPECT_TRUE(test_result.isApprox(SO3Hat(test_data)));
}

TEST(HatTest, HandleDoubleInput) {
    Eigen::Vector3f test_data(1.0, 2.0, 3.0);
    Eigen::Matrix3f test_result;
    test_result << 0.0, -3.0, 2.0,
            3.0, 0.0, -1.0,
            -2.0, 1.0, 0.0;

    EXPECT_TRUE(test_result.isApprox(SO3Hat(test_data)));
}

TEST(SO3ExpTest, HandleZeroInput) {
    Eigen::Vector3d test_data(0.0, 0.0, 0.0);

    Eigen::Matrix3d test_result;
    test_result << 1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0;

    EXPECT_TRUE(SO3Exp(test_data).isApprox(test_result));
}

TEST(SO3ExpTest, HandleFloatInput) {
    Eigen::Vector3f test_data(0.0f, 0.0f, 0.0f);

    Eigen::Matrix3f test_result;
    test_result << 1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f;

    EXPECT_TRUE(SO3Exp(test_data).isApprox(test_result));
}

TEST(SO3ExpTest, HandleFloatPi2Input) {
    Eigen::Vector3f test_data_0(1.0f, 0.0f, 0.0f);

    Eigen::Vector3f test_data(1.0f * M_PI_2, 0.0f, 0.0f);

    Eigen::Matrix3f test_result;
    test_result = test_data_0 * test_data_0.transpose() + 1.0f * SO3Hat(test_data_0);

    EXPECT_TRUE(SO3Exp(test_data).isApprox(test_result));
}

TEST(SO3ExpTest, HandleDoublePi2Input) {
    Eigen::Vector3d test_data_0(1.0, 0.0, 0.0);

    Eigen::Vector3d test_data(1.0 * M_PI_2, 0.0, 0.0);

    Eigen::Matrix3d test_result;
    test_result = test_data_0 * test_data_0.transpose() + 1.0 * SO3Hat(test_data_0);

    EXPECT_TRUE(SO3Exp(test_data).isApprox(test_result));
}

TEST(SO3ExpTest, HandleDoublePi4Input) {
    Eigen::Vector3d test_data_0(1.0, 0.0, 0.0);

    Eigen::Vector3d test_data(1.0 * M_PI_4, 0.0, 0.0);

    Eigen::Matrix3d test_result;
    test_result = std::cos(M_PI_4) * Eigen::Matrix3d::Identity()
                  + (1.0 - std::cos(M_PI_4)) * test_data_0 * test_data_0.transpose()
                  + std::sin(M_PI_4) * SO3Hat(test_data_0);

    EXPECT_TRUE(SO3Exp(test_data).isApprox(test_result));
}

TEST(SO3ExpTest, HandleDouble3PiInput) {
    Eigen::Vector3d test_data_0(1.0, 0.0, 0.0);

    Eigen::Vector3d test_data(1.0 * 3.0 * M_PI, 0.0, 0.0);

    Eigen::Matrix3d test_result;
    test_result = std::cos(M_PI) * Eigen::Matrix3d::Identity()
                  + (1.0 - std::cos(M_PI)) * test_data_0 * test_data_0.transpose()
                  + std::sin(M_PI) * SO3Hat(test_data_0);

    EXPECT_TRUE(SO3Exp(test_data).isApprox(test_result));
}

TEST(SO3ExpTest, HandleDoubleNegativeInput) {
    Eigen::Vector3d test_data_0(1.0, 0.0, 0.0);

    Eigen::Vector3d test_data(-1.0 * 3.0 * M_PI, 0.0, 0.0);

    Eigen::Matrix3d test_result;
    test_result = std::cos(M_PI) * Eigen::Matrix3d::Identity()
                  + (1.0 - std::cos(M_PI)) * test_data_0 * test_data_0.transpose()
                  + std::sin(M_PI) * SO3Hat(test_data_0);

    EXPECT_TRUE(SO3Exp(test_data).isApprox(test_result));
}

TEST(SO3ExpTest, HandleTranspose) {
    Eigen::Vector3d test_data(1.0, 0.0, 0.0);

    EXPECT_TRUE(SO3Exp(test_data).transpose().isApprox(SO3Exp(-test_data)));
}

TEST(SE3ExpTest, HandleNormalInput) {
    Vec6d epsilon_omega;
    epsilon_omega << 0.5, 1.0, 1.5, 0.1, 0.001, 0.00124;

    Mat4d T = SE3Exp(epsilon_omega);
    Mat4d T_true;
    T_true << 0.999998732257249, -0.0011879755058542, 0.00106028208140807, 0.500177323147329,
            0.00128789217916111, 0.99500339817527, -0.0998327549124039, 0.923714786259781,
            -0.000936385406507498, 0.0998339938791529, 0.995003666751288, 1.54722007984458,
            0, 0, 0, 1;

    EXPECT_TRUE(T.isApprox(T_true));
    EXPECT_TRUE(epsilon_omega.isApprox(SE3Log(T)));
}

TEST(Mod2PiTest, HandleAnyInput) {
    EXPECT_DOUBLE_EQ(Mod2Pi(0.0), 0.0);
    EXPECT_DOUBLE_EQ(Mod2Pi(M_PI), -M_PI);
    EXPECT_DOUBLE_EQ(Mod2Pi(-M_PI), -M_PI);
    EXPECT_DOUBLE_EQ(Mod2Pi(M_PI * 10.0), 0.0);
    EXPECT_DOUBLE_EQ(Mod2Pi(M_PI * 9.0), -M_PI);
    EXPECT_DOUBLE_EQ(Mod2Pi(M_PI * (-9.0)), -M_PI);
    EXPECT_DOUBLE_EQ(Mod2Pi(M_PI * (-8.0)), 0.0);
}

TEST(RotationMatrixToRPYTest, HandleInput) {
    auto RotateByX = [](double theta) -> Eigen::Matrix3d {
        Eigen::Matrix3d R;
        R << 1, 0, 0,
                0, cos(theta), -sin(theta),
                0, sin(theta), cos(theta);
        return R;
    };

    auto RotateByY = [](double theta) -> Eigen::Matrix3d {
        Eigen::Matrix3d R;
        R << cos(theta), 0, sin(theta),
                0, 1, 0,
                -sin(theta), 0, cos(theta);
        return R;
    };

    auto RotateByZ = [](double theta) -> Eigen::Matrix3d {
        Eigen::Matrix3d R;
        R << cos(theta), -sin(theta), 0,
                sin(theta), cos(theta), 0,
                0, 0, 1;
        return R;
    };

    const Eigen::Matrix3d R = RotateByZ(M_PI / 3.0) * RotateByY(M_PI_4) * RotateByX(M_PI / 6.0);

    const auto euler = RotationMatrixToRPY(R);

    EXPECT_DOUBLE_EQ(euler.x(), M_PI / 6.0);
    EXPECT_DOUBLE_EQ(euler.y(), M_PI / 4.0);
    EXPECT_DOUBLE_EQ(euler.z(), M_PI / 3.0);
}

TEST(So3JacobianLeftTest, TestFloatInput) {
    Eigen::Vector3f v(0.1f, 0.1f, 0.1f);

    Eigen::Matrix3f Jl = So3JacobianLeft(v);

    Eigen::Matrix3f Jl_true;
    Jl_true << 0.996672f, -0.048211f, 0.0515393f,
            0.0515393f, 0.996672f, -0.048211f,
            -0.048211f, 0.0515393f, 0.996672f;

    EXPECT_TRUE(Jl.isApprox(Jl_true));
}

TEST(So3JacobianLeftTest, TestDoubleInput) {
    Eigen::Vector3d v(0.1, 0.1, 0.1);

    Eigen::Matrix3d Jl = So3JacobianLeft(v);

    Eigen::Matrix3d Jl_true;
    Jl_true << 0.99667166309672583502, -0.048210956481420833009, 0.051539293384695025746,
            0.051539293384695025746, 0.99667166309672583502, -0.048210956481420833009,
            -0.048210956481420833009, 0.051539293384695025746, 0.99667166309672583502;

    EXPECT_TRUE(Jl.isApprox(Jl_true));
}

TEST(So3JacobianLeftTest, TestZeroInput) {
    Eigen::Vector3d v(1.0e-9, 1.0e-9, 1.0e-9);

    Eigen::Matrix3d Jl = So3JacobianLeft(v);

    Eigen::Matrix3d Jl_true = Eigen::Matrix3d::Identity();

    EXPECT_TRUE(Jl.isApprox(Jl_true));
}

TEST(So3JacobianRightTest, TestNormalInput) {
    Eigen::Vector3d v(1.0e-1, 1.0e-1, 1.0e-1);

    Eigen::Matrix3d Jr = So3JacobianRight(v);

    Eigen::Matrix3d Jr_true = So3JacobianLeft(-v);

    EXPECT_TRUE(Jr.isApprox(Jr_true));
}

TEST(SO3LogTest, TestNormalInput) {
    Eigen::Vector3d u(1.0, 1.0, 1.0);
    u.normalize();

    double theta = M_PI / 4.0;

    Eigen::Matrix3d R = SO3Exp(u * theta);

    EXPECT_TRUE(SO3Log(R).isApprox(u * theta));
}

TEST(SO3LogTest, TestZeroInput) {
    Eigen::Vector3d u(1.0, 1.0, 1.0);
    u.normalize();

    double theta = 0.0;

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    EXPECT_TRUE(SO3Log(R).isApprox(u * theta));
}

TEST(SO3LogTest, TestPiInput) {
    Eigen::Vector3d u(1.0, 1.0, 1.0);
    u.normalize();

    double theta = M_PI - 0.01;

    Eigen::Matrix3d R = SO3Exp(u * theta);

    EXPECT_TRUE(SO3Log(R).isApprox(u * theta));
}

TEST(SO3LogTest, Test2PiInput) {
    Eigen::Vector3d u(1.0, 1.0, 1.0);
    u.normalize();

    double theta = M_PI * 2.0;

    Eigen::Matrix3d R = SO3Exp(u * theta);

    EXPECT_TRUE(SO3Log(R).isApprox(
            Eigen::Vector3d(3.71380017872296178e-16, 3.71380017872296178e-16, 3.71380017872296178e-16)));
}
