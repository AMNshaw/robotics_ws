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
    int k_neighbors = 5;
    int max_iterations = 30;
    double convergence_threshold = 1e-5;
    int min_valid_points = 50;
    double degenerate_threshold = 100.0;

    Eigen::Isometry3d T_base_lidar = Eigen::Isometry3d::Identity();
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
    static Eigen::Isometry3d applyLieUpdate(const Eigen::Isometry3d& T,
                                            const Eigen::Matrix<double, 6, 1>& dx);

    static Eigen::Matrix<double, 6, 6> computeCovariance(const Eigen::Matrix<double, 6, 6>& H,
                                                         int n_valid_points);

    static bool checkDegenerate(const Eigen::Matrix<double, 6, 6>& H, double degenerate_threshold);

    core::IMapBuilder::SharedPtr map_builder_;
    IkdTreeScanMatcherParams params_;
    Eigen::Isometry3d T_base_lidar_ = Eigen::Isometry3d::Identity();
};

}  // namespace lio_slam_shaw::scan_matcher

#endif  // LIO_SLAM_SHAW__SCAN_MATCHER__IKD_TREE_SCAN_MATCHER_HPP_
