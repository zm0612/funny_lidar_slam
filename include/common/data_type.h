//
// Created by Zhang Zhimeng on 22-4-26.
//

#ifndef FUNNY_LIDAR_SLAM_DATA_TYPE_H
#define FUNNY_LIDAR_SLAM_DATA_TYPE_H

#include "lidar/lidar_point_type.h"

#include <vector>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template<int dim>
using TypeVectorVecd = typename std::vector<Eigen::Matrix<double, dim, 1>,
        Eigen::aligned_allocator<Eigen::Matrix<double, dim, 1>>>;

typedef uint64_t TimeStampUs;

typedef TypeVectorVecd<4> VectorVec4d;
typedef TypeVectorVecd<3> VectorVec3d;
typedef TypeVectorVecd<2> VectorVec2d;

typedef typename pcl::PointXYZ PCLPointXYZ;
typedef typename pcl::PointCloud<PCLPointXYZ> PCLPointCloudXYZ;

typedef typename pcl::PointXYZI PCLPointXYZI;
typedef typename pcl::PointCloud<PCLPointXYZI> PCLPointCloudXYZI;

typedef typename pcl::PointXY PCLPointXY;
typedef typename pcl::PointCloud<PCLPointXY> PCLPointCloudXY;
typedef typename pcl::PointCloud<PointXYZIRT> PCLPointCloudXYZIRT;
typedef typename std::vector<PCLPointXYZI, Eigen::aligned_allocator<PCLPointXYZI>> PCLPointVector;

typedef typename Eigen::Vector2d Vec2d;
typedef typename Eigen::Vector3d Vec3d;
typedef typename Eigen::Vector3f Vec3f;
typedef typename Eigen::Vector4f Vec4f;
typedef typename Eigen::Vector4d Vec4d;
typedef typename Eigen::Matrix<float, 5, 1> Vec5f;
typedef typename Eigen::Matrix<float, 6, 1> Vec6f;
typedef typename Eigen::Matrix<double, 6, 1> Vec6d;
typedef typename Eigen::Matrix<double, 9, 1> Vec9d;
typedef typename Eigen::Matrix<double, 15, 1> Vec15d;

typedef typename Eigen::Vector2i Vec2i;
typedef typename Eigen::Vector3i Vec3i;
typedef typename Eigen::VectorXi VecXi;

typedef typename Eigen::Matrix2d Mat2d;
typedef typename Eigen::Matrix3d Mat3d;
typedef typename Eigen::Matrix3f Mat3f;
typedef typename Eigen::Matrix4f Mat4f;
typedef typename Eigen::Matrix4d Mat4d;
typedef typename Eigen::Matrix<float, 5, 5> Mat5f;
typedef typename Eigen::Matrix<float, 6, 6> Mat6f;
typedef typename Eigen::Matrix<double, 6, 6> Mat6d;
typedef typename Eigen::Matrix<double, 9, 9> Mat9d;
typedef typename Eigen::Matrix<double, 15, 15> Mat15d;
typedef typename Eigen::Matrix<double, 24, 24> Mat24d;

typedef typename Eigen::MatrixXf MatXf;
typedef typename Eigen::MatrixXd MatXd;
typedef typename Eigen::VectorXf VecXf;
typedef typename Eigen::VectorXd VecXd;

#endif //FUNNY_LIDAR_SLAM_DATA_TYPE_H
