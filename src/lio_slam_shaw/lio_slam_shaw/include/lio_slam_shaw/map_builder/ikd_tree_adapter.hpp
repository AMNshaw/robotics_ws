#ifndef LIO_SLAM_SHAW__MAP_BUILDER__IKD_TREE_ADAPTER_HPP_
#define LIO_SLAM_SHAW__MAP_BUILDER__IKD_TREE_ADAPTER_HPP_

#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <shared_mutex>

#include "ikd_tree.h"
#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::map_builder {

struct NearestPlaneResult {
    bool valid = false;
    Eigen::Vector3d point_in_map;
    Eigen::Vector3d normal;
    Eigen::Vector3d centroid;
};

class IkdTreeAdapter {
public:
    using PointVector = KD_TREE<core::PointXYZIRT>::PointVector;

    struct Config {
        float ikd_delete_param = 0.5f;
        float ikd_balance_param = 0.6f;
        float ikd_downsample_size = 0.3f;

        int k_query_neighbors = 5;
        int min_plane_points = 3;
        double plane_valid_threshold = 0.1;
    };
    explicit IkdTreeAdapter(const Config& params);

    void addScan(const Eigen::Isometry3d& T_map_sensor,
                 const core::PointCloudIRTConstPtr& cloud_sensor);
    void boxTrim(const Eigen::Vector3d& center, double half_length);
    void clearMap();
    bool searchNearest(const core::PointXYZIRT& query_pt, int k, float search_dist,
                       const PointVector*& out_neighbors,
                       const std::vector<float>*& out_distances) const;
    NearestPlaneResult queryNearestPlane(const core::PointXYZIRT& query_pt, float search_dist);

private:
    NearestPlaneResult fitPlane(const PointVector& neighbors,
                                const Eigen::Vector3d& query_point_in_map) const;

    Config params_;

    bool is_first_frame_ = true;
    mutable std::shared_mutex ikd_tree_mutex_;
    std::shared_ptr<KD_TREE<core::PointXYZIRT>> ikd_tree_;

    int k_query_neighbors_ = 5;
    int min_plane_points_ = 3;
    double plane_valid_threshold_ = 0.1;
};

}  // namespace lio_slam_shaw::map_builder

#endif  // LIO_SLAM_SHAW__MAP_BUILDER__IKD_TREE_ADAPTER_HPP_