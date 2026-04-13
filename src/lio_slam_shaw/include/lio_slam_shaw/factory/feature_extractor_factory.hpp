#ifndef LIO_SLAM_SHAW__FACTORY__FEATURE_EXTRACTOR_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__FEATURE_EXTRACTOR_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_feature_extractor.hpp"

namespace lio_slam_shaw::factory {

class FeatureExtractorFactory {
public:
    static core::IFeatureExtractor::SharedPtr create(rclcpp::Node::SharedPtr node);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__FEATURE_EXTRACTOR_FACTORY_HPP_
