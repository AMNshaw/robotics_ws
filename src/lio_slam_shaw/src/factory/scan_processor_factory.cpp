#include "lio_slam_shaw/factory/scan_processor_factory.hpp"

#include "lio_slam_shaw/lidar_preprocessor/imu_deskew_preprocessor.hpp"

namespace lio_slam_shaw::factory {

core::IScanPreprocessor::SharedPtr ScanProcessorFactory::createScanPreprocessor(
    rclcpp::Node::SharedPtr node, const core::IImuPreintegrator::SharedPtr& imu_preintegrator) {
    std::string type = node->declare_parameter<std::string>("scan_preprocessor_type", "deskew");

    if (type == "deskew") {
        return createDeskewScanPreprocessor(imu_preintegrator);
    }
    RCLCPP_WARN(node->get_logger(),
                "Unknown scan_preprocessor_type '%s', falling back to passthrough.", type.c_str());
    return createDefaultScanPreprocessor();
}

core::IScanPreprocessor::SharedPtr ScanProcessorFactory::createDefaultScanPreprocessor() {
    // Passthrough: 直接回傳原始點雲，不做任何處理
    class PassthroughPreprocessor : public core::IScanPreprocessor {
    public:
        core::LidarData processCloud(const std::vector<core::ImuData>& /*imu_data*/,
                                     const core::LidarData& raw_data) override {
            return raw_data;
        }
    };
    return std::make_shared<PassthroughPreprocessor>();
}

core::IScanPreprocessor::SharedPtr ScanProcessorFactory::createDeskewScanPreprocessor(
    const core::IImuPreintegrator::SharedPtr& imu_preintegrator) {
    return std::make_shared<lidar_preprocessor::ImuDeskewPreprocessor>(imu_preintegrator);
}

}  // namespace lio_slam_shaw::factory