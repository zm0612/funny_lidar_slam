//
// Created by Zhang Zhimeng on 23-12-13.
//
#include "ivox_map/ivox_map.h"

bool IVoxMap::GetClosestPoint(const PCLPointXYZI &pt, PointVector &closest_pt, std::size_t max_num,
                              float max_range) {
    std::vector<DistPoint> candidates;
    candidates.reserve(max_num * nearby_grids_.size());

    Vec3i key = Pos2Grid(pt.getVector3fMap());

    for (const KeyType &delta: nearby_grids_) {
        Vec3i dkey = key + delta;
        auto iter = grids_map_.find(dkey);
        if (iter != grids_map_.end()) {
            iter->second->second.KNNPointByCondition(candidates, pt, max_num, max_range);
        }
    }

    if (candidates.empty()) {
        return false;
    }

    if (candidates.size() <= max_num) {
    } else {
        std::nth_element(candidates.begin(), candidates.begin() + max_num - 1, candidates.end());
        candidates.resize(max_num);
    }
    std::nth_element(candidates.begin(), candidates.begin(), candidates.end());

    closest_pt.clear();
    for (auto &it: candidates) {
        closest_pt.emplace_back(it.Get());
    }
    return closest_pt.empty() == false;
}

[[maybe_unused]] size_t IVoxMap::NumValidGrids() const {
    return grids_map_.size();
}

void IVoxMap::GenerateNearbyGrids() {
    if (options_.nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
        nearby_grids_ = {KeyType(0, 0, 0), KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY18) {
        nearby_grids_ = {KeyType(0, 0, 0), KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1), KeyType(1, 1, 0),
                         KeyType(-1, 1, 0), KeyType(1, -1, 0), KeyType(-1, -1, 0), KeyType(1, 0, 1),
                         KeyType(-1, 0, 1), KeyType(1, 0, -1), KeyType(-1, 0, -1), KeyType(0, 1, 1),
                         KeyType(0, -1, 1), KeyType(0, 1, -1), KeyType(0, -1, -1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY26) {
        nearby_grids_ = {KeyType(0, 0, 0), KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1), KeyType(1, 1, 0),
                         KeyType(-1, 1, 0), KeyType(1, -1, 0), KeyType(-1, -1, 0), KeyType(1, 0, 1),
                         KeyType(-1, 0, 1), KeyType(1, 0, -1), KeyType(-1, 0, -1), KeyType(0, 1, 1),
                         KeyType(0, -1, 1), KeyType(0, 1, -1), KeyType(0, -1, -1), KeyType(1, 1, 1),
                         KeyType(-1, 1, 1), KeyType(1, -1, 1), KeyType(1, 1, -1), KeyType(-1, -1, 1),
                         KeyType(-1, 1, -1), KeyType(1, -1, -1), KeyType(-1, -1, -1)};
    } else {
        LOG(ERROR) << "Unknown nearby_type!";
    }
}

[[maybe_unused]] void
IVoxMap::AddPoints(const PointVector &world_points, const std::vector<PointVector> &nearest_points) {
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    std::size_t cur_pts = world_points.size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }

    std::for_each(std::execution::unseq, index.begin(), index.end(), [&](const size_t &i) {
        const PCLPointXYZI &point_world = world_points[i];
        if (!nearest_points[i].empty()) {
            const PointVector &points_near = nearest_points[i];

            Eigen::Vector3f center = ((point_world.getVector3fMap() * options_.inv_resolution_).array().floor() + 0.5) *
                                     options_.resolution_;

            Eigen::Vector3f dis_to_center = points_near[0].getVector3fMap() - center;

            // if nearest point is very close to current point, don't need to add it.
            if (fabs(dis_to_center.x()) > 0.5 * options_.resolution_ &&
                fabs(dis_to_center.y()) > 0.5 * options_.resolution_ &&
                fabs(dis_to_center.z()) > 0.5 * options_.resolution_) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }

            bool need_add = true;
            float dist_square = (point_world.getVector3fMap() - center).squaredNorm();
            if (points_near.size() >= options_.num_nearest_points_) {
                for (size_t readd_i = 0; readd_i < options_.num_nearest_points_; readd_i++) {
                    if ((points_near[readd_i].getVector3fMap() - center).squaredNorm() < dist_square + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    });

    AddPoints(points_to_add);
    AddPoints(point_no_need_downsample);
}

void IVoxMap::AddPoints(const PointVector &points_to_add) {
    std::for_each(std::execution::unseq, points_to_add.begin(), points_to_add.end(), [this](const auto &pt) {
        auto key = Pos2Grid(pt.getVector3fMap());

        auto iter = grids_map_.find(key);
        if (iter == grids_map_.end()) {
            grids_cache_.push_front({key, NodeType()});
            grids_map_.insert({key, grids_cache_.begin()});

            grids_cache_.front().second.InsertPoint(pt);

            if (grids_map_.size() >= options_.capacity_) {
                grids_map_.erase(grids_cache_.back().first);
                grids_cache_.pop_back();
            }
        } else {
            iter->second->second.InsertPoint(pt);
            grids_cache_.splice(grids_cache_.begin(), grids_cache_, iter->second);
            grids_map_[key] = grids_cache_.begin();
        }
    });
}

Vec3i IVoxMap::Pos2Grid(const IVoxMap::PtType &pt) const {
    return (pt * options_.inv_resolution_).array().round().cast<int>();
}

[[maybe_unused]] std::vector<float> IVoxMap::StatGridPoints() const {
    std::size_t num = grids_cache_.size(), valid_num = 0, max = 0, min = 100000000;
    std::size_t sum = 0, sum_square = 0;

    for (auto &it: grids_cache_) {
        std::size_t s = it.second.Size();
        valid_num += s > 0;
        max = s > max ? s : max;
        min = s < min ? s : min;
        sum += s;
        sum_square += s * s;
    }

    float ave = static_cast<float>(sum) / static_cast<float>(num);
    float stddev = num > 1 ?
                   std::sqrt(
                           (static_cast<float>(sum_square) - static_cast<float>(num) * ave * ave) /
                           (static_cast<float>(num) - 1)
                   ) : 0;
    return std::vector<float>{static_cast<float>(valid_num), ave, static_cast<float>(max),
                              static_cast<float>(min), stddev};
}