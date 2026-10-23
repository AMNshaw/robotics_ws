#pragma once

#include <Eigen/Dense>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw {

struct ScanContextParams {
    int num_ring = 20;
    int num_sector = 60;
    double max_radius = 80.0;
    double lidar_height = 2.0;
    int num_candidates = 10;  // top-K ring key pre-filter inside findClosest
    double sc_dist_threshold = 0.13;
    double search_ratio = 0.1;           // half-range for circular shift refinement
    bool enable_reverse_search = false;  // SC++: also try column-reversed descriptor
};

/// SC descriptor returned by getDescriptor().
struct ScDescriptor {
    Eigen::MatrixXf mat;          ///< SC matrix (num_ring x num_sector)
    std::vector<float> ring_key;  ///< row-wise mean, for fast pre-filtering
};

/// Pure-computation Scan Context engine.  Stateless.
class ScanContextManager {
public:
    explicit ScanContextManager(const ScanContextParams& params = {});

    /// Compute SC descriptor from a PCL cloud (body/LiDAR frame).
    ScDescriptor getDescriptor(const core::PointCloudIRTConstPtr& cloud) const;

    /// Find best match from a candidate set.
    /// Internally: ring-key L2 top-K pre-filter -> pairwise SC distance with shift alignment.
    /// Returns {matched_id, yaw_diff_rad} or nullopt if none passes sc_dist_threshold.
    std::optional<std::pair<uint64_t, float>> findClosest(
        const ScDescriptor& query,
        const std::unordered_map<uint64_t, ScDescriptor>& candidates) const;

    const ScanContextParams& params() const { return params_; }

private:
    Eigen::VectorXf makeRingKey(const Eigen::MatrixXf& desc) const;
    Eigen::RowVectorXf makeSectorKey(const Eigen::MatrixXf& desc) const;
    std::pair<double, int> distanceBtnSC(const Eigen::MatrixXf& sc1,
                                         const Eigen::MatrixXf& sc2) const;
    double distDirectSC(const Eigen::MatrixXf& sc1, const Eigen::MatrixXf& sc2) const;
    int fastAlignUsingVkey(const Eigen::RowVectorXf& vk1, const Eigen::RowVectorXf& vk2) const;
    static Eigen::MatrixXf circshift(const Eigen::MatrixXf& mat, int n);

    ScanContextParams params_;
};

}  // namespace lio_slam_shaw
