#include "lio_slam_shaw/factory/scan_preprocessor_factory.hpp"

#include "lio_slam_shaw/lidar_preprocessor/imu_deskew_preprocessor.hpp"

namespace lio_slam_shaw::factory {

core::IScanPreprocessor::SharedPtr ScanPreprocessorFactory::create(
    rclcpp::Node::SharedPtr node, const core::IImuPreintegrator::SharedPtr& imu_preintegrator) {
    std::string type = node->declare_parameter<std::string>("scan_preprocessor_type", "deskew");

    if (type == "deskew") {
        return createDeskew(imu_preintegrator);
    }
    RCLCPP_WARN(node->get_logger(),
                "Unknown scan_preprocessor_type '%s', falling back to passthrough.", type.c_str());
    return createDefault();
}

core::IScanPreprocessor::SharedPtr ScanPreprocessorFactory::createDefault() {
    class PassthroughPreprocessor : public core::IScanPreprocessor {
    public:
        core::LidarData processCloud(const std::vector<core::ImuData>& /*imu_data*/,
                                     const core::LidarData& raw_data) override {
            return raw_data;
        }
    };
    return std::make_shared<PassthroughPreprocessor>();
}

core::IScanPreprocessor::SharedPtr ScanPreprocessorFactory::createDeskew(
    const core::IImuPreintegrator::SharedPtr& imu_preintegrator) {
    return std::make_shared<lidar_preprocessor::ImuDeskewPreprocessor>(imu_preintegrator);
}

}  // namespace lio_slam_shaw::factory