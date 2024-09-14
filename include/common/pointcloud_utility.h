//
// Created by Zhang Zhimeng on 22-11-29.
//

#ifndef FUNNY_LIDAR_SLAM_POINTCLOUD_UTILITY_H
#define FUNNY_LIDAR_SLAM_POINTCLOUD_UTILITY_H

#include "common/data_type.h"
#include <pcl/filters/voxel_grid.h>
#include <glog/logging.h>
#include <execution>

template <typename PointT>
inline float DistanceSquared(const PointT& pt1, const PointT& pt2) {
    Eigen::Vector3f d = pt1.getVector3fMap() - pt2.getVector3fMap();
    return d.squaredNorm();
}

template <typename T, int dim, typename PointType>
inline Eigen::Matrix<T, dim, 1> PclPointToEigen(const PointType& pt) {
    return Eigen::Matrix<T, dim, 1>(pt.x, pt.y, pt.z);
}

template <>
inline Eigen::Matrix<float, 3, 1> PclPointToEigen<float, 3, pcl::PointXYZ>(const pcl::PointXYZ& pt) {
    return pt.getVector3fMap();
}

template <>
inline Eigen::Matrix<float, 3, 1> PclPointToEigen<float, 3, pcl::PointXYZI>(const pcl::PointXYZI& pt) {
    return pt.getVector3fMap();
}

template <>
inline Eigen::Matrix<float, 3, 1> PclPointToEigen<float, 3, PointXYZIRT>(const PointXYZIRT& pt) {
    return pt.getVector3fMap();
}


inline PCLPointXYZ TransformPoint(const PCLPointXYZ& pt, const Mat3d& R,
                                  const Vec3d& t) {
    const Vec3f temp = R.cast<float>() * Vec3f(pt.x, pt.y, pt.z) + t.cast<float>();
    return {temp.x(), temp.y(), temp.z()};
}

inline PCLPointXYZ TransformPoint(const PCLPointXYZ& pt, const Mat3f& R,
                                  const Vec3f& t) {
    const Vec3f temp = R * Vec3f(pt.x, pt.y, pt.z) + t;
    return {temp.x(), temp.y(), temp.z()};
}

inline PCLPointXYZI TransformPoint(const PCLPointXYZI& pt, const Mat3d& R,
                                   const Vec3d& t) {
    const Vec3f temp = R.cast<float>() * Vec3f(pt.x, pt.y, pt.z) + t.cast<float>();
    PCLPointXYZI pt_trans;
    pt_trans.x = temp.x();
    pt_trans.y = temp.y();
    pt_trans.z = temp.z();
    pt_trans.intensity = pt.intensity;
    return pt_trans;
}

inline PCLPointXYZI TransformPoint(const PCLPointXYZI& pt, const Mat3f& R,
                                   const Vec3f& t) {
    const Vec3f temp = R * Vec3f(pt.x, pt.y, pt.z) + t;
    PCLPointXYZI pt_trans;
    pt_trans.x = temp.x();
    pt_trans.y = temp.y();
    pt_trans.z = temp.z();
    pt_trans.intensity = pt.intensity;
    return pt_trans;
}

inline PointXYZIRT TransformPoint(const PointXYZIRT& pt, const Mat3f& R,
                                  const Vec3f& t) {
    const Vec3f temp = R * Vec3f(pt.x, pt.y, pt.z) + t;
    PointXYZIRT pt_trans{};
    pt_trans.x = temp.x();
    pt_trans.y = temp.y();
    pt_trans.z = temp.z();
    pt_trans.intensity = pt.intensity;
    pt_trans.time = pt.time;
    pt_trans.ring = pt.ring;
    return pt_trans;
}

inline PCLPointCloudXYZ TransformPointCloud(const PCLPointCloudXYZ& cloud, const Mat3f& R, const Vec3f& t) {
    const unsigned int N = cloud.size();

    PCLPointCloudXYZ cloud_trans;
    cloud_trans.resize(cloud.size());
    for (unsigned int i = 0; i < N; ++i) {
        cloud_trans[i] = TransformPoint(cloud[i], R, t);
    }

    return cloud_trans;
}

inline PCLPointCloudXYZ TransformPointCloud(const PCLPointCloudXYZ& cloud, const Mat3d& R, const Vec3d& t) {
    const unsigned int N = cloud.size();

    const Mat3f R_temp = R.cast<float>();
    const Vec3f t_temp = t.cast<float>();

    PCLPointCloudXYZ cloud_trans;
    cloud_trans.resize(cloud.size());
    for (unsigned int i = 0; i < N; ++i) {
        cloud_trans[i] = TransformPoint(cloud[i], R_temp, t_temp);
    }

    return cloud_trans;
}

inline PCLPointCloudXYZI TransformPointCloud(const PCLPointCloudXYZI& cloud, const Mat3f& R, const Vec3f& t) {
    const unsigned int N = cloud.size();

    PCLPointCloudXYZI cloud_trans;
    cloud_trans.resize(cloud.size());
    for (unsigned int i = 0; i < N; ++i) {
        cloud_trans[i] = TransformPoint(cloud[i], R, t);
    }

    return cloud_trans;
}

inline PCLPointCloudXYZI TransformPointCloud(const PCLPointCloudXYZI& cloud, const Mat3d& R, const Vec3d& t) {
    const unsigned int N = cloud.size();

    const Mat3f R_temp = R.cast<float>();
    const Vec3f t_temp = t.cast<float>();

    PCLPointCloudXYZI cloud_trans;
    cloud_trans.resize(cloud.size());
    for (unsigned int i = 0; i < N; ++i) {
        cloud_trans[i] = TransformPoint(cloud[i], R_temp, t_temp);
    }

    return cloud_trans;
}

inline PCLPointCloudXYZI::Ptr TransformPointCloud(const PCLPointCloudXYZI::Ptr& cloud, const Mat4d& T) {
    CHECK_NOTNULL(cloud);
    const unsigned int N = cloud->size();

    const Mat3f R_temp = T.block<3, 3>(0, 0).cast<float>();
    const Vec3f t_temp = T.block<3, 1>(0, 3).cast<float>();

    PCLPointCloudXYZI::Ptr cloud_trans(new PCLPointCloudXYZI);
    cloud_trans->resize(cloud->size());

    std::vector<size_t> indices(N);
    std::iota(indices.begin(), indices.end(), 0);
    std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i) {
        cloud_trans->points[i] = TransformPoint(cloud->points[i], R_temp, t_temp);
    });

    return cloud_trans;
}

inline PCLPointCloudXYZI::Ptr TransformPointCloud(const PCLPointCloudXYZI::Ptr& cloud, const Mat4f& T) {
    CHECK_NOTNULL(cloud);
    const unsigned int N = cloud->size();

    const Mat3f R_temp = T.block<3, 3>(0, 0);
    const Vec3f t_temp = T.block<3, 1>(0, 3);

    PCLPointCloudXYZI::Ptr cloud_trans(new PCLPointCloudXYZI);
    cloud_trans->resize(cloud->size());

    std::vector<size_t> indices(N);
    std::iota(indices.begin(), indices.end(), 0);
    std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i) {
        cloud_trans->points[i] = TransformPoint(cloud->points[i], R_temp, t_temp);
    });

    return cloud_trans;
}

inline PCLPointCloudXYZI TransformPointCloud(const PCLPointCloudXYZI& cloud, const Mat4d& T) {
    const unsigned int N = cloud.size();

    const Mat3f R_temp = T.block<3, 3>(0, 0).cast<float>();
    const Vec3f t_temp = T.block<3, 1>(0, 3).cast<float>();

    PCLPointCloudXYZI::Ptr cloud_trans(new PCLPointCloudXYZI);
    cloud_trans->resize(cloud.size());

    std::vector<size_t> indices(N);
    std::iota(indices.begin(), indices.end(), 0);
    std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i) {
        cloud_trans->points[i] = TransformPoint(cloud.points[i], R_temp, t_temp);
    });

    return *cloud_trans;
}

inline PCLPointCloudXYZIRT TransformPointCloud(const PCLPointCloudXYZIRT& cloud, const Mat4d& T) {
    const unsigned int N = cloud.size();

    const Mat3f R_temp = T.block<3, 3>(0, 0).cast<float>();
    const Vec3f t_temp = T.block<3, 1>(0, 3).cast<float>();

    PCLPointCloudXYZIRT::Ptr cloud_trans(new PCLPointCloudXYZIRT);
    cloud_trans->header = cloud.header;
    cloud_trans->resize(cloud.size());

    std::vector<size_t> indices(N);
    std::iota(indices.begin(), indices.end(), 0);
    std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i) {
        cloud_trans->points[i] = TransformPoint(cloud.points[i], R_temp, t_temp);
    });

    return *cloud_trans;
}

inline PCLPointCloudXYZI::Ptr VoxelGridCloud(const PCLPointCloudXYZI::Ptr& cloud, float voxel_size = 0.1) {
    pcl::VoxelGrid<PCLPointXYZI> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);

    PCLPointCloudXYZI::Ptr output(new PCLPointCloudXYZI);
    voxel.filter(*output);
    return output;
}

template <typename PointT>
inline void RemoveNaNFromPointCloud(const pcl::PointCloud<PointT>& cloud_in, pcl::PointCloud<PointT>& cloud_out) {
    // If the clouds are not the same, prepare the output
    if (&cloud_in != &cloud_out) {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
        cloud_out.sensor_origin_ = cloud_in.sensor_origin_;
        cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
    }

    // If the data is dense, we don't need to check for NaN
    if (cloud_in.is_dense) {
        // Simply copy the data
        cloud_out = cloud_in;
    } else {
        std::size_t j = 0;
        for (std::size_t i = 0; i < cloud_in.points.size(); ++i) {
            if (!std::isfinite(cloud_in.points[i].x) ||
                !std::isfinite(cloud_in.points[i].y) ||
                !std::isfinite(cloud_in.points[i].z))
                continue;
            cloud_out.points[j] = cloud_in.points[i];
            j++;
        }
        if (j != cloud_in.points.size()) {
            // Resize to the correct size
            cloud_out.points.resize(j);
        }

        cloud_out.height = 1;
        cloud_out.width = static_cast<std::uint32_t>(j);

        // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
        cloud_out.is_dense = true;
    }
}

inline PCLPointCloudXYZI::Ptr VoxelGridCloud(const PCLPointCloudXYZI& cloud, float voxel_size = 0.1) {
    pcl::VoxelGrid<PCLPointXYZI> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud.makeShared());

    PCLPointCloudXYZI::Ptr output(new PCLPointCloudXYZI);
    voxel.filter(*output);
    return output;
}

#endif //FUNNY_LIDAR_SLAM_POINTCLOUD_UTILITY_H
