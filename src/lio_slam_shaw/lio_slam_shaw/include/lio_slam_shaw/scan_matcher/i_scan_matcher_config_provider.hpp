#ifndef LIO_SLAM_SHAW__SCAN_MATCHER__I_SCAN_MATCHER_CONFIG_PROVIDER_HPP_
#define LIO_SLAM_SHAW__SCAN_MATCHER__I_SCAN_MATCHER_CONFIG_PROVIDER_HPP_

#include <variant>

#include "lio_slam_shaw/core/i_config_provider.hpp"
#include "lio_slam_shaw/scan_matcher/ikd_tree_scan_matcher.hpp"

namespace lio_slam_shaw::scan_matcher {

using ScanMatcherConfig = std::variant<IkdTreeScanMatcher::Config>;

class IIkdTreeScanMatcherConfigProvider : core::IConfigProvider<IkdTreeScanMatcher::Config> {
public:
    virtual ~IIkdTreeScanMatcherConfigProvider() = default;

    virtual IkdTreeScanMatcher::Config load() const = 0;
};

}  // namespace lio_slam_shaw::scan_matcher

#endif  // LIO_SLAM_SHAW__SCAN_MATCHER__I_SCAN_MATCHER_CONFIG_PROVIDER_HPP_
