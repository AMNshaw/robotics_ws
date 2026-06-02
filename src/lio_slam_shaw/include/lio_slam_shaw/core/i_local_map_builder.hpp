#ifndef LIO_SLAM_SHAW__CORE__I_LOCAL_MAP_BUILDER_HPP_
#define LIO_SLAM_SHAW__CORE__I_LOCAL_MAP_BUILDER_HPP_

#include <Eigen/Geometry>
#include <memory>

#include "lio_slam_shaw/core/lidar_frame.hpp"
#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {

/// Local map for scan-to-map odometry. Stores points in an incrementally-updated
/// spatial index (ikd-tree). Bounded by box trimming — never rebuilt.
class ILocalMapBuilder {
public:
    using SharedPtr = std::shared_ptr<ILocalMapBuilder>;
    using ConstSharedPtr = std::shared_ptr<const ILocalMapBuilder>;

    virtual ~ILocalMapBuilder() = default;

    /// Add a scan's points to the local map (transformed to world frame internally).
    /// Internally performs box trimming around the current position.
    virtual void addScan(const LidarFrame::SharedPtr& frame) = 0;

    /// Returns true once the spatial index has been built (at least one scan added).
    virtual bool isMapReady() const = 0;
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__I_LOCAL_MAP_BUILDER_HPP_
