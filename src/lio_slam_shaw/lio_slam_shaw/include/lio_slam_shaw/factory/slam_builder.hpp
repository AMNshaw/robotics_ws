#ifndef LIO_SLAM_SHAW__FACTORY__SLAM_BUILDER_HPP_
#define LIO_SLAM_SHAW__FACTORY__SLAM_BUILDER_HPP_

#include <memory>

#include "lio_slam_shaw/core/backend.hpp"
#include "lio_slam_shaw/core/frontend.hpp"
#include "lio_slam_shaw/core/sensor_data_types.hpp"
#include "lio_slam_shaw/core/slam_processor.hpp"
#include "lio_slam_shaw/factory/frontend_config_provider.hpp"
#include "lio_slam_shaw/initializer/i_lio_initializer_config_provider.hpp"
#include "lio_slam_shaw/loop_closure_detector/i_loop_closure_detector_config_provider.hpp"
#include "lio_slam_shaw/map_builder/i_map_builder_config_provider.hpp"
#include "lio_slam_shaw/map_optimizer/i_map_optimizer_config_provider.hpp"

namespace lio_slam_shaw::factory {

class SlamBuilder {
public:
    SlamBuilder(const core::Extrinsics ex) : extrinsics_(ex) {}

    SlamBuilder* buildFrontEnd(const FrontEndConfig& config) {
        std::visit(
            [this](auto&& arg) {
                using ConfigType = std::decay_t<decltype(arg)>;

                if constexpr (std::is_same_v<ConfigType, FastLioFrontEndConfig>) {
                    assert(initializer_ != nullptr && "initializer_ is null");

                    const auto& cfg = static_cast<const FastLioFrontEndConfig&>(arg);
                    lidar_preprocessor::ImuDeskewPreprocessor::SharedPtr deskew =
                        std::make_shared<lidar_preprocessor::ImuDeskewPreprocessor>(
                            cfg.preprocessor_config);
                    feature_extractor::PassthroughFeatureExtractor::SharedPtr passthrough =
                        std::make_shared<feature_extractor::PassthroughFeatureExtractor>();
                    map_builder::IkdTreeLocalMapBuilder::SharedPtr ikd_map =
                        std::make_shared<map_builder::IkdTreeLocalMapBuilder>(
                            cfg.local_map_builder_config);
                    odometry_estimator::FastLioOdometry::SharedPtr fast_lio =
                        std::make_shared<odometry_estimator::FastLioOdometry>(
                            extrinsics_.T_base_lidar, extrinsics_.T_base_imu,
                            cfg.odometry_estimator_config);

                    core::FrontEndParams frontend_cfg = {cfg.max_pending_lidar_queue};

                    frontend_ = std::make_unique<core::FrontEnd>(
                        std::move(deskew), std::move(passthrough), std::move(fast_lio),
                        std::move(ikd_map), std::move(initializer_), frontend_cfg);

                } else {
                    static_assert(always_false_v<ConfigType>, "Non-exhaustive visitor!");
                }
            },
            config);
        return this;
    }

    SlamBuilder* buildInitializer(const initializer::InitializerConfig& config) {
        using namespace initializer;
        std::visit(
            [this](auto&& arg) {
                using ConfigType = std::decay_t<decltype(arg)>;

                if constexpr (std::is_same_v<ConfigType, SfmLioInitializer::Config>) {
                    const auto& cfg = static_cast<const SfmLioInitializer::Config&>(arg);
                    Eigen::Isometry3d T_imu_lidar =
                        extrinsics_.T_base_imu.inverse() * extrinsics_.T_base_lidar;
                    initializer_ = std::make_shared<SfmLioInitializer>(
                        cfg.scan_matcher_params, cfg.map_builder_params, T_imu_lidar, cfg);
                } else {
                    static_assert(always_false_v<ConfigType>, "Non-exhaustive visitor!");
                }
            },
            config);
        return this;
    }

    SlamBuilder* buildMapOptimizer(const map_optimizer::MapOptimizerConfig& config) {
        using namespace map_optimizer;
        std::visit(
            [this](auto&& arg) {
                using ConfigType = std::decay_t<decltype(arg)>;

                if constexpr (std::is_same_v<ConfigType, GtsamMapOptimizer::Config>) {
                    const auto& cfg = static_cast<const GtsamMapOptimizer::Config&>(arg);
                    map_optimizer_ = std::make_shared<GtsamMapOptimizer>(cfg);
                } else {
                    static_assert(always_false_v<ConfigType>, "Non-exhaustive visitor!");
                }
            },
            config);
        return this;
    }

    SlamBuilder* buildGlobalMap(const map_builder::GlobalMapConfig& config) {
        using namespace map_builder;
        std::visit(
            [this](auto&& arg) {
                using ConfigType = std::decay_t<decltype(arg)>;

                if constexpr (std::is_same_v<ConfigType, KeyframeGlobalMapBuilder::Config>) {
                    const auto& cfg = static_cast<const KeyframeGlobalMapBuilder::Config&>(arg);
                    global_map_ = std::make_shared<KeyframeGlobalMapBuilder>(cfg);

                } else {
                    static_assert(always_false_v<ConfigType>, "Non-exhaustive visitor!");
                }
            },
            config);

        return this;
    }

    SlamBuilder* buildLoopClosureDetector(
        const loop_closure_detector::LoopClosureDetectorConfig& config) {
        using namespace loop_closure_detector;
        std::visit(
            [this](auto&& arg) {
                using ConfigType = std::decay_t<decltype(arg)>;

                if constexpr (std::is_same_v<ConfigType, ScanContextLoopClosureDetector::Config>) {
                    loop_closure_ = std::make_shared<ScanContextLoopClosureDetector>(arg);
                } else if constexpr (std::is_same_v<ConfigType, IcpLoopClosureDetector::Config>) {
                    loop_closure_ = std::make_shared<IcpLoopClosureDetector>(arg);
                } else {
                    static_assert(always_false_v<ConfigType>, "Non-exhaustive visitor!");
                }
            },
            config);
        return this;
    }

    SlamBuilder* buildBackEnd() {
        assert(global_map_ != nullptr && "global_map_ is null");
        assert(loop_closure_ != nullptr && "loop_closure_ is null");
        assert(map_optimizer_ != nullptr && "map_optimizer_ is null");

        backend_ = std::make_unique<core::BackEnd>(
            std::move(global_map_), std::move(map_optimizer_), std::move(loop_closure_));

        return this;
    }

    core::SlamProcessor::UniquePtr buildSlamProcressor() {
        assert(frontend_ && "frontend_ is null");
        assert(backend_ && "backend_ is null");

        return std::make_unique<core::SlamProcessor>(std::move(frontend_), std::move(backend_));
    }

private:
    core::Extrinsics extrinsics_;

    core::ILioInitializer::SharedPtr initializer_ = nullptr;
    core::IGlobalMapBuilder::SharedPtr global_map_ = nullptr;
    core::ILoopClosureDetector::SharedPtr loop_closure_ = nullptr;
    core::IMapOptimizer::SharedPtr map_optimizer_ = nullptr;

    core::BackEnd::UniquePtr backend_ = nullptr;
    core::FrontEnd::UniquePtr frontend_ = nullptr;

    template <class>
    inline static constexpr bool always_false_v = false;
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__SLAM_BUILDER_HPP_