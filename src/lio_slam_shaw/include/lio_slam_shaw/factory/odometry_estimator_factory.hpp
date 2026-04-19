#ifndef LIO_SLAM_SHAW__FACTORY__ODOMETRY_ESTIMATOR_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__ODOMETRY_ESTIMATOR_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_map_builder.hpp"
#include "lio_slam_shaw/core/i_odometry_estimator.hpp"

namespace lio_slam_shaw::factory {

class OdometryEstimatorFactory {
public:
    static core::IOdometryEstimator::SharedPtr create(rclcpp::Node* node,
                                                      const Eigen::Isometry3d& T_base_lidar,
                                                      const Eigen::Isometry3d& T_base_imu,
                                                      core::IMapBuilder::SharedPtr map_builder);

private:
    static core::IOdometryEstimator::SharedPtr createFastLio(
        rclcpp::Node* node, const Eigen::Isometry3d& T_base_lidar,
        const Eigen::Isometry3d& T_base_imu, core::IMapBuilder::SharedPtr map_builder);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__ODOMETRY_ESTIMATOR_FACTORY_HPP_
