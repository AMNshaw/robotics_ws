#ifndef LIO_SLAM_SHAW_ROS2__FACTORY__FRONTEND_FACADE_HPP_
#define LIO_SLAM_SHAW_ROS2__FACTORY__FRONTEND_FACADE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/factory/frontend_facade.hpp"

namespace lio_slam_shaw_ros2::factory {

namespace core = lio_slam_shaw::core;
namespace lidar_preprocessor = lio_slam_shaw::lidar_preprocessor;
namespace feature_extractor = lio_slam_shaw::feature_extractor;
namespace map_builder = lio_slam_shaw::map_builder;
namespace odometry_estimator = lio_slam_shaw::odometry_estimator;

class RosFastLioFrontEndFacade : public lio_slam_shaw::factory::IFastLioFrontEndFacade {
public:
    RosFastLioFrontEndFacade(rclcpp::Node* node) : node_(node) {}

private:
    lidar_preprocessor::ImuDeskewPreprocessor::SharedPtr createScanPreprocessor(
        const Eigen::Isometry3d& T_base_lidar) const override;
    feature_extractor::PassthroughFeatureExtractor::SharedPtr createFeatureExtractor()
        const override;
    map_builder::IkdTreeLocalMapBuilder::SharedPtr createLocalMapBuilder(
        const Eigen::Isometry3d& T_base_lidar) const override;
    odometry_estimator::FastLioOdometry::SharedPtr createOdometryEstimator(
        const Eigen::Isometry3d& T_base_lidar, const Eigen::Isometry3d& T_base_imu,
        core::ILocalMapBuilder::SharedPtr local_map) const override;

    rclcpp::Node* node_;
};

}  // namespace lio_slam_shaw_ros2::factory

#endif  // LIO_SLAM_SHAW_ROS2__FACTORY__FRONTEND_FACADE_HPP_