#include "lio_slam_shaw/factory/slam_factory.hpp"

#include "lio_slam_shaw/core/backend.hpp"
#include "lio_slam_shaw/core/frontend.hpp"
#include "lio_slam_shaw/core/sensor_data_manager.hpp"
#include "lio_slam_shaw/factory/feature_extractor_factory.hpp"
#include "lio_slam_shaw/factory/imu_preintegrator_factory.hpp"
#include "lio_slam_shaw/factory/map_builder_factory.hpp"
#include "lio_slam_shaw/factory/map_optimizer_factory.hpp"
#include "lio_slam_shaw/factory/scan_matcher_factory.hpp"
#include "lio_slam_shaw/factory/scan_preprocessor_factory.hpp"

namespace lio_slam_shaw::factory {

namespace {

// ── 暫時 stub：ILoopClosureDetector ────────────────────────────────────────
// TODO: 替換為 concrete IcpLoopClosureDetector 實作
class StubLoopClosureDetector : public core::ILoopClosureDetector {
public:
    std::optional<core::LoopConstraint> detect(const core::KeyFrame::SharedPtr&,
                                               core::IMapBuilder::SharedPtr) override {
        return std::nullopt;
    }
};

}  // namespace

core::SlamProcessor::SharedPtr SlamFactory::create(rclcpp::Node::SharedPtr node) {
    // 1. IMU preintegrator（所有需要 IMU 狀態的元件都依賴它）
    auto imu_preintegrator = ImuPreintegratorFactory::create(node);

    // 2. Scan preprocessor
    auto scan_preprocessor = ScanPreprocessorFactory::create(node);

    // 3. Feature extractor
    auto feature_extractor = FeatureExtractorFactory::create(node);

    // 4. Map builder
    auto map_builder = MapBuilderFactory::create(node);

    // 5. Scan matcher
    auto scan_matcher = ScanMatcherFactory::create(node, map_builder);

    // 6. Sensor data manager
    auto sensor_data_manager = std::make_shared<core::SensorDataManager>();

    // 7. FrontEnd
    auto frontend = std::make_shared<core::FrontEnd>(
        sensor_data_manager, scan_preprocessor, feature_extractor, scan_matcher, imu_preintegrator);

    // 8. Map optimizer & loop closure detector
    // TODO: 替換為 IcpLoopClosureDetector
    auto map_optimizer = MapOptimizerFactory::create(node);
    auto loop_closure_detector = std::make_shared<StubLoopClosureDetector>();

    // 9. BackEnd
    auto backend =
        std::make_shared<core::BackEnd>(map_builder, map_optimizer, loop_closure_detector);

    // 10. SlamProcessor
    return std::make_shared<core::SlamProcessor>(frontend, backend);
}

}  // namespace lio_slam_shaw::factory
