#ifndef LIO_SLAM_SHAW__ODOMETRY_ESTIMATOR__I_ODOMETRY_ESTIMATOR_FACTORY_HPP_
#define LIO_SLAM_SHAW__ODOMETRY_ESTIMATOR__I_ODOMETRY_ESTIMATOR_FACTORY_HPP_

#include "lio_slam_shaw/core/i_odometry_estimator.hpp"
#include "lio_slam_shaw/map_builder/ikd_tree_local_map_builder.hpp"

namespace lio_slam_shaw::odometry_estimator {

enum class OdometryEstimatorType { FAST_LIO };

class IOdometryEstimatorFactory {
public:
    virtual ~IOdometryEstimatorFactory() = default;
    virtual core::IOdometryEstimator::SharedPtr create(const Eigen::Isometry3d& T_base_lidar,
                                                       const Eigen::Isometry3d& T_base_imu,
                                                       core::ILocalMapBuilder::SharedPtr local_map,
                                                       OdometryEstimatorType type) const = 0;

private:
    virtual core::IOdometryEstimator::SharedPtr createFastLio(
        const Eigen::Isometry3d& T_base_lidar, const Eigen::Isometry3d& T_base_imu,
        std::shared_ptr<map_builder::IkdTreeLocalMapBuilder> local_map) const = 0;
};

}  // namespace lio_slam_shaw::odometry_estimator

#endif  // LIO_SLAM_SHAW__ODOMETRY_ESTIMATOR__I_ODOMETRY_ESTIMATOR_FACTORY_HPP_
