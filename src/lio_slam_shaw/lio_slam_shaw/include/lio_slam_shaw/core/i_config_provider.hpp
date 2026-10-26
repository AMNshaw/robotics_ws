#ifndef LIO_SLAM_SHAW__CORE__I_CONFIG_PROVIDER_HPP_
#define LIO_SLAM_SHAW__CORE__I_CONFIG_PROVIDER_HPP_

namespace lio_slam_shaw::core {

template <typename TConfig>
class IConfigProvider {
public:
    virtual ~IConfigProvider() = default;
    virtual TConfig load() const = 0;
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__I_CONFIG_PROVIDER_HPP_