#ifndef LIO_SLAM_SHAW__SCAN_MATCHER__IKD_TREE_SCAN_MATCHER_HPP_
#define LIO_SLAM_SHAW__SCAN_MATCHER__IKD_TREE_SCAN_MATCHER_HPP_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "lio_slam_shaw/core/i_map_builder.hpp"
#include "lio_slam_shaw/core/i_scan_matcher.hpp"
#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::scan_matcher {

struct IkdTreeScanMatcherParams {
    // 每個 query 點查詢的最近鄰數量（用於擬合局部平面）
    int k_neighbors = 5;
    // GN 最大迭代次數
    int max_iterations = 30;
    // δx 的 L2 norm 低於此值則宣告收斂
    double convergence_threshold = 1e-5;
    // 一次迭代中最少有效點對數，低於此值視為退化
    int min_valid_points = 50;
    // Hessian 最大/最小特徵值比值超過此值則視為退化環境
    double degenerate_threshold = 100.0;
};

// GN/LM point-to-plane scan matcher，使用 IMapBuilder 提供的 ikd-Tree 查詢最近鄰
// 數學上等同於 FAST-LIO 的 iEKF，但用 Gauss-Newton 迭代求解 6-DoF 位姿
//
// 殘差模型 (point-to-plane):
//   r_i = n_i^T * (T * p_i^body - c_i)
// 其中 n_i 為平面法向量，c_i 為鄰近點重心，p_i^body 為 lidar 系下的 query 點
//
// Jacobian (left perturbation, xi = [δt; δφ] ∈ se(3)):
//   dr_i/dxi = [ n_i^T,  (T*p_i^body × n_i)^T ]
//
// 求解: H * δx = -b, 其中 H = Σ J_i^T J_i, b = Σ J_i^T r_i
// 更新: T ← exp(δx^) * T  (Lie algebra update)
class IkdTreeScanMatcher : public core::IScanMatcher {
public:
    using SharedPtr = std::shared_ptr<IkdTreeScanMatcher>;

    explicit IkdTreeScanMatcher(core::IMapBuilder::SharedPtr map_builder,
                                const IkdTreeScanMatcherParams& params = {});

    // 以 initial_guess.pose 為初始狀態，對 features.raw_deskewed 進行 GN 迭代匹配
    // 回傳 ScanMatchResult，其中 pose 為地圖座標系下的最終位姿
    core::ScanMatchResult match(const core::FeatureSet& features,
                                const core::NavState& initial_guess) override;

private:
    // 將 6-vector Lie algebra δx = [δt; δφ] 轉換為 SE(3) 增量並左乘到 T
    static Eigen::Isometry3d applyLieUpdate(const Eigen::Isometry3d& T,
                                            const Eigen::Matrix<double, 6, 1>& dx);

    // 計算近似 6x6 點雲協方差，用於 ScanMatchResult::covariance
    // H 為最後一次 GN 迭代的 Hessian 估計 (J^T J)
    static Eigen::Matrix<double, 6, 6> computeCovariance(const Eigen::Matrix<double, 6, 6>& H,
                                                         int n_valid_points);

    // 利用 Hessian 最小特徵值比值判斷是否退化（例如長廊、空曠環境）
    static bool checkDegenerate(const Eigen::Matrix<double, 6, 6>& H, double degenerate_threshold);

    core::IMapBuilder::SharedPtr map_builder_;
    IkdTreeScanMatcherParams params_;
};

}  // namespace lio_slam_shaw::scan_matcher

#endif  // LIO_SLAM_SHAW__SCAN_MATCHER__IKD_TREE_SCAN_MATCHER_HPP_
