#ifndef LIO_SLAM_SHAW__LOOP_CLOSURE_DETECTOR__I_LOOP_CLOSURE_DETECTOR_CONFIG_PROVIDER_HPP_
#define LIO_SLAM_SHAW__LOOP_CLOSURE_DETECTOR__I_LOOP_CLOSURE_DETECTOR_CONFIG_PROVIDER_HPP_

#include <variant>

#include "lio_slam_shaw/core/i_config_provider.hpp"
#include "lio_slam_shaw/loop_closure_detector/icp_loop_closure_detector.hpp"
#include "lio_slam_shaw/loop_closure_detector/scan_context_loop_closure_detector.hpp"

namespace lio_slam_shaw::loop_closure_detector {

using LoopClosureDetectorConfig =
    std::variant<IcpLoopClosureDetector::Config, ScanContextLoopClosureDetector::Config>;

class IScanContextLoopClosureDetectorConfigProvider
    : core::IConfigProvider<ScanContextLoopClosureDetector::Config> {
public:
    virtual ~IScanContextLoopClosureDetectorConfigProvider() = default;
    virtual ScanContextLoopClosureDetector::Config load() const = 0;
};

class IIcpLoopClosureDetectorConfigProvider
    : core::IConfigProvider<IcpLoopClosureDetector::Config> {
public:
    virtual ~IIcpLoopClosureDetectorConfigProvider() = default;
    virtual IcpLoopClosureDetector::Config load() const = 0;
};

}  // namespace lio_slam_shaw::loop_closure_detector

#endif  // LIO_SLAM_SHAW__LOOP_CLOSURE_DETECTOR__I_LOOP_CLOSURE_DETECTOR_CONFIG_PROVIDER_HPP_
