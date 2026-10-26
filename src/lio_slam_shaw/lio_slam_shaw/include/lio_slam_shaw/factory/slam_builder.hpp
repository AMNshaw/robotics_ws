#ifndef LIO_SLAM_SHAW__FACTORY__SLAM_BUILDER_HPP_
#define LIO_SLAM_SHAW__FACTORY__SLAM_BUILDER_HPP_

#include <memory>

#include "lio_slam_shaw/core/backend.hpp"
#include "lio_slam_shaw/core/frontend.hpp"
#include "lio_slam_shaw/core/slam_processor.hpp"
#include "lio_slam_shaw/factory/frontend_config_provider.hpp"
#include "lio_slam_shaw/initializer/i_lio_initializer_config_provider.hpp"
#include "lio_slam_shaw/loop_closure_detector/i_loop_closure_detector_config_provider.hpp"
#include "lio_slam_shaw/map_builder/i_map_builder_config_provider.hpp"
#include "lio_slam_shaw/map_optimizer/i_map_optimizer_config_provider.hpp"

namespace lio_slam_shaw::factory {

class SlamBuilder {
public:
    SlamBuilder() = default;

    SlamBuilder* buildFrontEnd(const FrontEndConfig& config) {
        std::visit(
            [this](auto&& arg) {
                using ConfigType = std::decay_t<decltype(arg)>;

                if constexpr (std::is_same_v<ConfigType, FastLioFrontEndConfig>) {
                } else {
                    static_assert(always_false_v<ConfigType>, "Non-exhaustive visitor!");
                }
            },
            config);
        return this;
    }

    SlamBuilder* buildInitializer(const initializer::InitializerConfig& conf) {}

    SlamBuilder* buildMapOptimizer(const map_optimizer::MapOptimizerConfig& conf) {}

    SlamBuilder* buildGlobalMap(const map_builder::GlobalMapConfig& config) {}

    SlamBuilder* buildLoopClosureDetector(
        const loop_closure_detector::LoopClosureDetectorConfig& config) {
        using namespace loop_closure_detector;
        std::visit(
            [this](auto&& arg) {
                using ConfigType = std::decay_t<decltype(arg)>;

                if constexpr (std::is_same_v<ConfigType, ScanContextLoopClosureDetector::Config>) {
                    loop_closure_ = std::make_unique<ScanContextLoopClosureDetector>(arg);
                } else if constexpr (std::is_same_v<ConfigType, IcpLoopClosureDetector::Config>) {
                    loop_closure_ = std::make_unique<IcpLoopClosureDetector>(arg);
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

    core::SlamProcessor::UniquePtr buildSlamProcressor();

private:
    core::ILioInitializer::UniquePtr initializer_ = nullptr;
    core::IGlobalMapBuilder::SharedPtr global_map_ = nullptr;
    core::ILoopClosureDetector::UniquePtr loop_closure_ = nullptr;
    core::IMapOptimizer::UniquePtr map_optimizer_ = nullptr;

    core::BackEnd::UniquePtr backend_ = nullptr;
    core::FrontEnd::UniquePtr frontend_ = nullptr;

    template <class>
    inline static constexpr bool always_false_v = false;
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__SLAM_BUILDER_HPP_