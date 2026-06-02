#ifndef LIO_SLAM_SHAW__FACTORY__BACKEND_FACADE_HPP_
#define LIO_SLAM_SHAW__FACTORY__BACKEND_FACADE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/backend.hpp"
#include "lio_slam_shaw/factory/slam_factory.hpp"

namespace lio_slam_shaw::factory {

// ---------------------------------------------------------------------------
// Interface: IBackEndFacade
//
// Encapsulates one coherent BackEnd stack (global map + optimizer +
// loop closure detector).  Algorithm-selection for loop closure is still
// YAML-configurable; the Facade only enforces that the combination is valid.
// ---------------------------------------------------------------------------
class IBackEndFacade {
public:
    virtual ~IBackEndFacade() = default;
    virtual core::BackEnd::SharedPtr create(rclcpp::Node* node, const Extrinsics& extrinsics) = 0;
};

// ---------------------------------------------------------------------------
// GtsamBackEndFacade
//
// Hardwired stack:
//   GlobalMapBuilder : IKD_TREE
//   MapOptimizer     : GTSAM (iSAM2)
//   LoopClosure      : type from YAML (ikd_tree | icp)
// ---------------------------------------------------------------------------
class GtsamBackEndFacade : public IBackEndFacade {
public:
    core::BackEnd::SharedPtr create(rclcpp::Node* node, const Extrinsics& extrinsics) override;
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__BACKEND_FACADE_HPP_
