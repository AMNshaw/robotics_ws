#ifndef LIO_SLAM_SHAW_ROS2__FACTORY__FEATURE_EXTRACTOR_FACTORY_HPP_
#define LIO_SLAM_SHAW_ROS2__FACTORY__FEATURE_EXTRACTOR_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/feature_extractor/i_feature_extractor_factory.hpp"

namespace lio_slam_shaw_ros2::factory {

class RosFeatureExtractorFactory
    : public lio_slam_shaw::feature_extractor::IFeatureExtractorFactory {
public:
    RosFeatureExtractorFactory(rclcpp::Node* node);

    lio_slam_shaw::core::IFeatureExtractor::SharedPtr create(
        lio_slam_shaw::feature_extractor::FeatureExtractorType type) const override;

private:
    rclcpp::Node* node_;
};

}  // namespace lio_slam_shaw_ros2::factory

#endif  // LIO_SLAM_SHAW_ROS2__FACTORY__FEATURE_EXTRACTOR_FACTORY_HPP_
