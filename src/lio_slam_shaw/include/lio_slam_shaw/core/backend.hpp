#ifndef LIO_SLAM_SHAW__CORE__BACKEND_HPP_
#define LIO_SLAM_SHAW__CORE__BACKEND_HPP_

#include <optional>
#include <utility>

#include "lio_slam_shaw/core/i_loop_closure_detector.hpp"
#include "lio_slam_shaw/core/i_map_builder.hpp"
#include "lio_slam_shaw/core/i_map_optimizer.hpp"
#include "lio_slam_shaw/core/lidar_frame.hpp"

namespace lio_slam_shaw::core {

class BackEnd {
public:
    using SharedPtr = std::shared_ptr<BackEnd>;
    using ConstSharedPtr = std::shared_ptr<const BackEnd>;

    BackEnd(IMapBuilder::SharedPtr map_builder, IMapOptimizer::SharedPtr map_optimizer,
            ILoopClosureDetector::SharedPtr loop_closure_detector);
    ~BackEnd() = default;

    // 處理新幀：插入地圖、加入 odometry edge、偵測 loop closure
    // loop closure 後將修正資訊存入 pending，等待 SlamProcessor 持鎖後統一套用
    void processKeyframe(const KeyFrame::SharedPtr& frame);

    // 取出並清除 pending correction delta（T_corrected * T_original⁻¹）
    // 若無 loop closure 則回傳 nullopt
    std::optional<Eigen::Isometry3d> updateGlobalCorrection();

    // 將 pending corrected poses 寫入 MapBuilder（ikd-Tree 重建）
    void updateMap();

private:
    IMapBuilder::SharedPtr map_builder_;
    IMapOptimizer::SharedPtr map_optimizer_;
    ILoopClosureDetector::SharedPtr loop_closure_detector_;

    // loop closure 後暫存，等 SlamProcessor 持 map_mutex_ 後再套用
    std::optional<Eigen::Isometry3d> pending_correction_;
    std::vector<std::pair<uint64_t, Eigen::Isometry3d>> pending_corrected_poses_;
};

}  // namespace lio_slam_shaw::core
#endif  // LIO_SLAM_SHAW__CORE__BACKEND_HPP_