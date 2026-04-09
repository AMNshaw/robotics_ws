#ifndef LIO_SLAM_SHAW__CORE__I_SCAN_MATCHER_HPP_
#define LIO_SLAM_SHAW__CORE__I_SCAN_MATCHER_HPP_

#include <memory>

#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {
class IScanMatcher {
public:
    using SharedPtr = std::shared_ptr<IScanMatcher>;
    using ConstSharedPtr = std::shared_ptr<const IScanMatcher>;
    IScanMatcher() = default;
    virtual ~IScanMatcher() = default;

    virtual ScanMatchResult match(const FeatureSet& features, const NavState& initial_guess) = 0;
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__I_SCAN_MATCHER_HPP_