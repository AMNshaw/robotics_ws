#ifndef LIO_SLAM_SHAW__FACTORY__FEATURE_EXTRACTOR_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__FEATURE_EXTRACTOR_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_feature_extractor.hpp"

namespace lio_slam_shaw::factory {

class FeatureExtractorFactory {
public:
    // type 由 ROS param "feature_extractor_type" 控制
    // 支援: "passthrough" (預設)
    static core::IFeatureExtractor::SharedPtr create(rclcpp::Node::SharedPtr node);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__FEATURE_EXTRACTOR_FACTORY_HPP_
