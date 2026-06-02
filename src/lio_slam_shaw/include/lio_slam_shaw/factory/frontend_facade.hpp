#ifndef LIO_SLAM_SHAW__FACTORY__FRONTEND_FACADE_HPP_
#define LIO_SLAM_SHAW__FACTORY__FRONTEND_FACADE_HPP_

#include <Eigen/Geometry>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/frontend.hpp"
#include "lio_slam_shaw/factory/slam_factory.hpp"
#include "lio_slam_shaw/map_builder/ikd_tree_local_map_builder.hpp"

namespace lio_slam_shaw::factory {

// ---------------------------------------------------------------------------
// Interface: IFrontEndFacade
//
// A Facade encapsulates one coherent FrontEnd algorithm stack (preprocessor +
// feature extractor + odometry estimator + local map), ensuring that only
// compatible combinations are assembled.  Algorithm-selection parameters are
// NOT exposed to users via ROS params; tuning parameters (noise, voxel size,
// etc.) still are.
// ---------------------------------------------------------------------------
class IFrontEndFacade {
public:
    virtual ~IFrontEndFacade() = default;

    // Build and return the fully-wired FrontEnd.
    virtual core::FrontEnd::SharedPtr create(rclcpp::Node* node, const Extrinsics& extrinsics) = 0;
};

// ---------------------------------------------------------------------------
// FastLIOFacade
//
// Hardwired stack:
//   ScanPreprocessor : IMU_DESKEW
//   FeatureExtractor : PASSTHROUGH
//   LocalMapBuilder  : IKD_TREE
//   OdometryEstimator: FAST_LIO
// ---------------------------------------------------------------------------
class FastLIOFacade : public IFrontEndFacade {
public:
    core::FrontEnd::SharedPtr create(rclcpp::Node* node, const Extrinsics& extrinsics) override;

private:
    std::shared_ptr<map_builder::IkdTreeLocalMapBuilder> local_map_;
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__FRONTEND_FACADE_HPP_
