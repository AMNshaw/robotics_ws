#ifndef LIO_SLAM_SHAW__INITIALIZER__I_LIO_INITIALIZER_CONFIG_PROVIDER_HPP_
#define LIO_SLAM_SHAW__INITIALIZER__I_LIO_INITIALIZER_CONFIG_PROVIDER_HPP_

#include <variant>

#include "lio_slam_shaw/core/i_config_provider.hpp"
#include "lio_slam_shaw/initializer/sfm_lio_initializer.hpp"

namespace lio_slam_shaw::initializer {

using InitializerConfig = std::variant<SfmLioInitializer::Config>;

class ISfmLioInitializerConfigProvider : core::IConfigProvider<SfmLioInitializer::Config> {
public:
    virtual ~ISfmLioInitializerConfigProvider() = default;
    virtual SfmLioInitializer::Config load() const = 0;
};

}  // namespace lio_slam_shaw::initializer

#endif  // LIO_SLAM_SHAW__INITIALIZER__I_LIO_INITIALIZER_CONFIG_PROVIDER_HPP_
