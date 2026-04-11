#ifndef LIO_SLAM_SHAW__LOOP_CLOSURE__ICP_LOOP_CLOSURE_DETECTOR_HPP_
#define LIO_SLAM_SHAW__LOOP_CLOSURE__ICP_LOOP_CLOSURE_DETECTOR_HPP_

#include <memory>
#include <optional>

#include "lio_slam_shaw/core/i_loop_closure_detector.hpp"

namespace lio_slam_shaw {

struct IcpLoopClosureDetectorParams {
    // 第一層：候選篩選
    double search_radius = 15.0;      // 空間搜尋半徑 (m)
    double min_time_gap_sec = 30.0;   // 與當前幀的最小時間差，避免剛走過的幀誤觸發
    int local_map_keyframe_num = 25;  // 建構局部地圖時，取候選附近幾幀

    // 第二層：ICP 幾何驗證
    float icp_downsample_leaf = 0.4f;  // VoxelGrid downsample leaf size (m)
    double icp_max_corr_dist = 0.3;    // ICP max correspondence distance (m)
    int icp_max_iterations = 100;
    double fitness_score_threshold = 0.3;  // 接受 loop 的 fitness 門檻（越小越嚴格）

    // LoopConstraint noise：對角 sigmas [rot(3), trans(3)]，fitness 越差倍率越大
    double noise_sigma_rot = 0.1;
    double noise_sigma_trans = 0.1;
};

class IcpLoopClosureDetector : public core::ILoopClosureDetector {
public:
    explicit IcpLoopClosureDetector(const IcpLoopClosureDetectorParams& params);
    ~IcpLoopClosureDetector() override = default;

    std::optional<core::LoopConstraint> detect(const core::KeyFrame::SharedPtr& current_keyframe,
                                               core::IMapBuilder::SharedPtr map_builder) override;

private:
    // 用 getAllKeyframes() 做 radius search，加上時間間隔過濾，取最近的一幀候選
    std::optional<core::KeyFrame::SharedPtr> findCandidate(
        const core::KeyFrame::SharedPtr& current_keyframe,
        const std::vector<core::KeyFrame::SharedPtr>& all_keyframes) const;

    // 取候選附近 local_map_keyframe_num 幀的點雲，組成局部地圖（world frame）
    core::PointCloudIRTPtr buildLocalMap(
        const core::KeyFrame::SharedPtr& current_keyframe,
        const core::KeyFrame::SharedPtr& candidate,
        const std::vector<core::KeyFrame::SharedPtr>& all_keyframes) const;

    IcpLoopClosureDetectorParams params_;
};

}  // namespace lio_slam_shaw

#endif  // LIO_SLAM_SHAW__LOOP_CLOSURE__ICP_LOOP_CLOSURE_DETECTOR_HPP_
