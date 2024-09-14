#ifndef FUNNY_LIDAR_SLAM_IVOX_IVOX_MAP_H
#define FUNNY_LIDAR_SLAM_IVOX_IVOX_MAP_H

#include "ivox_map/voxel_grid_node.h"
#include "common/hash_function.h"

#include <glog/logging.h>
#include <execution>
#include <list>
#include <thread>

/*!
 * Incremental Voxels Map
 * note: Simplify Incremental-Voxels-Map from faster lio
 */
class IVoxMap {
public:
    using KeyType = Eigen::Matrix<int, 3, 1>;
    using PtType = Eigen::Matrix<float, 3, 1>;
    using NodeType = VoxelGridNode;
    using PointVector = std::vector<PCLPointXYZI, Eigen::aligned_allocator<PCLPointXYZI>>;
    using DistPoint = typename NodeType::DistPoint;

    enum class NearbyType {
        CENTER,  // center only
        NEARBY6,
        NEARBY18,
        NEARBY26,
    };

    struct Options {
        float resolution_ = 0.2;                        // ivox resolution
        float inv_resolution_ = 10.0;                   // inverse resolution
        NearbyType nearby_type_ = NearbyType::NEARBY6;  // nearby range
        std::size_t capacity_ = 1000000;                // capacity
        std::size_t num_nearest_points_ = 5;
    };

    /**
     * constructor
     * @param options  ivox options
     */
    explicit IVoxMap(Options options) : options_(options) {
        options_.inv_resolution_ = 1.0f / options_.resolution_;
        GenerateNearbyGrids();
    }

    void AddPoints(const PointVector &points_to_add);

    /*!
     * Add Points based on adjacency
     * @param world_points world frame points
     * @param nearest_points world frame points
     */
    [[maybe_unused]] void AddPoints(const PointVector &world_points, const std::vector<PointVector> &nearest_points);

    bool GetClosestPoint(const PCLPointXYZI &pt, PointVector &closest_pt,
                         std::size_t max_num = 5, float max_range = 5.0);

    [[maybe_unused]] [[nodiscard]] size_t NumValidGrids() const;

    [[maybe_unused]] [[nodiscard]] std::vector<float> StatGridPoints() const;

private:
    void GenerateNearbyGrids();

    [[nodiscard]] KeyType Pos2Grid(const PtType &pt) const;

    Options options_{};
    std::unordered_map<KeyType,
            typename std::list<std::pair<KeyType, NodeType>>::iterator, SpatialHashFunction> grids_map_{};
    std::list<std::pair<KeyType, NodeType>> grids_cache_{};
    std::vector<KeyType> nearby_grids_{};
};

#endif
