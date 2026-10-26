#ifndef LIO_SLAM_SHAW_ROS2__FACTORY__FEATURE_EXTRACTOR_FACTORY_HPP_
#define LIO_SLAM_SHAW_ROS2__FACTORY__FEATURE_EXTRACTOR_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/feature_extractor/i_feature_extractor_factory.hpp"

namespace lio_slam_shaw_ros2::factory {

namespace core = lio_slam_shaw::core;
namespace feature_extractor = lio_slam_shaw::feature_extractor;

class RosFeatureExtractorFactory : public feature_extractor::IFeatureExtractorFactory {
public:
    RosFeatureExtractorFactory(rclcpp::Node* node) : node_(node) {}

    feature_extractor::IFeatureExtractor::SharedPtr create(
        feature_extractor::FeatureExtractorType type) const override;

private:
    rclcpp::Node* node_;
};

}  // namespace lio_slam_shaw_ros2::factory

#endif  // LIO_SLAM_SHAW_ROS2__FACTORY__FEATURE_EXTRACTOR_FACTORY_HPP_
