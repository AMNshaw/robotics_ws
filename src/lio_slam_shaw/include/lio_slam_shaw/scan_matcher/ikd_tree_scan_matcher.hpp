#ifndef LIO_SLAM_SHAW__SCAN_MATCHER__IKD_TREE_SCAN_MATCHER_HPP_
#define LIO_SLAM_SHAW__SCAN_MATCHER__IKD_TREE_SCAN_MATCHER_HPP_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "lio_slam_shaw/core/i_map_builder.hpp"
#include "lio_slam_shaw/core/i_scan_matcher.hpp"
#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::scan_matcher {

struct IkdTreeScanMatcherParams {
    int k_neighbors = 3;
    int max_iterations = 10;
    double convergence_threshold = 1e-3;
    int min_valid_points = 50;
    double degenerate_threshold = 100.0;

    float max_search_dist = 1.0;
    int min_plane_points = 3;
    double plane_valid_threshold = 0.1;

    /// Rotation regularization: penalise deviation from IMU-predicted rotation.
    /// sigma in radians. 0 = disabled.
    double rot_regularization_sigma = 0.0;

    Eigen::Isometry3d T_base_lidar = Eigen::Isometry3d::Identity();
};

struct NearestPlaneResult {
    bool valid = false;
    Eigen::Vector3d point_in_map;
    Eigen::Vector3d normal;
    Eigen::Vector3d centroid;
};

class IkdTreeScanMatcher : public core::IScanMatcher {
public:
    using SharedPtr = std::shared_ptr<IkdTreeScanMatcher>;

    explicit IkdTreeScanMatcher(core::IMapBuilder::SharedPtr map_builder,
                                const IkdTreeScanMatcherParams& params = {});

    void setLidarExtrinsics(const Eigen::Isometry3d& T_base_lidar) override;
    core::ScanMatchResult match(const core::FeatureSet& features,
                                const core::NavState& initial_guess) override;

private:
    Eigen::Isometry3d applyLieUpdate(const Eigen::Isometry3d& T,
                                     const Eigen::Matrix<double, 6, 1>& dx,
                                     const Eigen::Vector3d& rot_center);

    static Eigen::Matrix<double, 6, 6> computeCovariance(const Eigen::Matrix<double, 6, 6>& H,
                                                         int n_valid_points);

    static bool checkDegenerate(const Eigen::Matrix<double, 6, 6>& H, double degenerate_threshold);

    NearestPlaneResult fitPlane(const std::vector<lio_slam_shaw::core::PointXYZIRT>& neighbors,
                                const Eigen::Vector3d& query_point_in_map) const;

    core::IMapBuilder::SharedPtr map_builder_;
    IkdTreeScanMatcherParams params_;
    Eigen::Isometry3d T_base_lidar_ = Eigen::Isometry3d::Identity();
};

}  // namespace lio_slam_shaw::scan_matcher

#endif  // LIO_SLAM_SHAW__SCAN_MATCHER__IKD_TREE_SCAN_MATCHER_HPP_
