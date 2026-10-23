#pragma once

#include <Eigen/Geometry>
#include <utility>
#include <vector>

#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {

/// Loop closure edge for visualization.
struct LoopEdge {
    uint64_t from_id;
    uint64_t to_id;
    Eigen::Vector3d from_position;
    Eigen::Vector3d to_position;
    double fitness_score;
};

/// Local visualization (frontend rate): current scan in odom frame.
struct LocalVizData {
    Timestamp timestamp;
    Eigen::Isometry3d pose_odom;
    PointCloudIRTConstPtr scan;
};

/// Global visualization (backend rate): map correction + keyframe graph.
struct GlobalVizData {
    Eigen::Isometry3d T_map_odom;
    std::vector<std::pair<uint64_t, Eigen::Isometry3d>> keyframe_poses;
    std::vector<LoopEdge> loop_edges;
    PointCloudIRTPtr global_map;  // nullptr unless freshly assembled
};

}  // namespace lio_slam_shaw::core
