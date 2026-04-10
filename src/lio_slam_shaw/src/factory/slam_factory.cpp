#include "lio_slam_shaw/factory/slam_factory.hpp"

#include "lio_slam_shaw/core/backend.hpp"
#include "lio_slam_shaw/core/frontend.hpp"
#include "lio_slam_shaw/core/sensor_data_manager.hpp"
#include "lio_slam_shaw/factory/feature_extractor_factory.hpp"
#include "lio_slam_shaw/factory/imu_preintegrator_factory.hpp"
#include "lio_slam_shaw/factory/scan_matcher_factory.hpp"
#include "lio_slam_shaw/factory/scan_preprocessor_factory.hpp"
#include "lio_slam_shaw/map_builder/ikd_tree_map_builder.hpp"

namespace lio_slam_shaw::factory {

namespace {

// ── 暫時 stub：IMapOptimizer ────────────────────────────────────────────────
// TODO: 替換為 concrete GtsamMapOptimizer 實作
class StubMapOptimizer : public core::IMapOptimizer {
public:
    void addKeyframe(uint64_t, const core::ScanMatchResult&) override {}
    void addLoopConstraint(const core::LoopConstraint&) override {}
    std::vector<std::pair<uint64_t, Eigen::Isometry3d>> optimize() override { return {}; }
    core::NavState getKeyframePose(uint64_t) const override { return {}; }
};

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

    // 2. Scan preprocessor（deskew 需要 imu_preintegrator）
    auto scan_preprocessor = ScanPreprocessorFactory::create(node, imu_preintegrator);

    // 3. Feature extractor
    auto feature_extractor = FeatureExtractorFactory::create(node);

    // 4. Map builder
    map_builder::IkdTreeMapBuilderParams mb_params;
    mb_params.keyframe_distance_threshold = node->declare_parameter<double>(
        "map_builder.keyframe_distance_threshold", mb_params.keyframe_distance_threshold);
    mb_params.keyframe_angle_threshold = node->declare_parameter<double>(
        "map_builder.keyframe_angle_threshold", mb_params.keyframe_angle_threshold);
    mb_params.ikd_downsample_size = static_cast<float>(node->declare_parameter<double>(
        "map_builder.ikd_downsample_size", mb_params.ikd_downsample_size));
    mb_params.max_search_dist =
        node->declare_parameter<double>("map_builder.max_search_dist", mb_params.max_search_dist);
    auto map_builder = std::make_shared<map_builder::IkdTreeMapBuilder>(mb_params);

    // 5. Scan matcher
    auto scan_matcher = ScanMatcherFactory::create(node, map_builder);

    // 6. Sensor data manager
    auto sensor_data_manager = std::make_shared<core::SensorDataManager>();

    // 7. FrontEnd
    auto frontend = std::make_shared<core::FrontEnd>(
        sensor_data_manager, scan_preprocessor, feature_extractor, scan_matcher, imu_preintegrator);

    // 8. Map optimizer & loop closure detector
    // TODO: 替換為 GtsamMapOptimizer, IcpLoopClosureDetector
    auto map_optimizer = std::make_shared<StubMapOptimizer>();
    auto loop_closure_detector = std::make_shared<StubLoopClosureDetector>();

    // 9. BackEnd
    auto backend =
        std::make_shared<core::BackEnd>(map_builder, map_optimizer, loop_closure_detector);

    // 10. SlamProcessor
    return std::make_shared<core::SlamProcessor>(frontend, backend);
}

}  // namespace lio_slam_shaw::factory
