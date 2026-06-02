#include "lio_slam_shaw/factory/scan_preprocessor_factory.hpp"

#include "lio_slam_shaw/lidar_preprocessor/imu_deskew_preprocessor.hpp"

namespace lio_slam_shaw::factory {

core::IScanPreprocessor::SharedPtr ScanPreprocessorFactory::create(
    rclcpp::Node* node, const Eigen::Isometry3d& T_base_lidar, ScanPreprocessorType type) {
    switch (type) {
        case ScanPreprocessorType::IMU_DESKEW:
            return createDeskew(node, T_base_lidar);
    }
    throw std::invalid_argument("ScanPreprocessorFactory: unknown type");
}

core::IScanPreprocessor::SharedPtr ScanPreprocessorFactory::createDeskew(
    rclcpp::Node* node, const Eigen::Isometry3d& T_base_lidar) {
    lidar_preprocessor::ImuDeskewPreprocessorParams params;
    params.voxel_leaf_size = static_cast<float>(
        node->declare_parameter<double>("frontend.scan_preprocessor.voxel_leaf_size", 0.2));
    params.T_base_lidar = T_base_lidar;
    return std::make_shared<lidar_preprocessor::ImuDeskewPreprocessor>(params);
}

}  // namespace lio_slam_shaw::factory