#include "lio_slam_shaw/scan_matcher/ikd_tree_scan_matcher.hpp"

#include <Eigen/Dense>
#include <cmath>

namespace lio_slam_shaw::scan_matcher {

IkdTreeScanMatcher::IkdTreeScanMatcher(core::IMapBuilder::SharedPtr map_builder,
                                       const IkdTreeScanMatcherParams& params)
    : map_builder_(std::move(map_builder)), params_(params) {
    // 建構 T_body_lidar
    const auto& t = params.T_body_lidar_trans;
    const auto& q = params.T_body_lidar_rot;  // [qx, qy, qz, qw]
    T_body_lidar_ = Eigen::Isometry3d::Identity();
    T_body_lidar_.linear() =
        Eigen::Quaterniond(q[3], q[0], q[1], q[2]).normalized().toRotationMatrix();
    T_body_lidar_.translation() = Eigen::Vector3d(t[0], t[1], t[2]);
}

core::ScanMatchResult IkdTreeScanMatcher::match(const core::FeatureSet& features,
                                                const core::NavState& initial_guess) {
    core::ScanMatchResult result;
    result.pose = initial_guess.pose;

    const auto& cloud = features.raw_deskewed;
    if (!cloud || cloud->empty()) {
        return result;
    }

    Eigen::Matrix<double, 6, 6> H_final = Eigen::Matrix<double, 6, 6>::Zero();
    int n_valid_final = 0;

    for (int iter = 0; iter < params_.max_iterations; ++iter) {
        // 1. 以當前位姿估計查詢 k 個最近鄰，並取得擬合平面
        // T_map_lidar = T_map_body * T_body_lidar
        const Eigen::Isometry3d T_map_lidar = result.pose * T_body_lidar_;
        auto nn_results = map_builder_->queryNearestPoints(cloud, T_map_lidar, params_.k_neighbors);

        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
        int n_valid = 0;

        for (const auto& nn : nn_results) {
            if (!nn.valid) continue;

            // 殘差: r = n^T * (p_map - c)
            double r = nn.normal.dot(nn.point_in_map - nn.centroid);

            // Jacobian: dr/dxi = [ n^T, (p_map × n)^T ]
            // (left perturbation on SE(3), xi = [δt; δφ])
            Eigen::Matrix<double, 6, 1> J;
            J.head<3>() = nn.normal;
            J.tail<3>() = nn.point_in_map.cross(nn.normal);

            H += J * J.transpose();
            b += J * r;
            ++n_valid;
        }

        if (n_valid < params_.min_valid_points) {
            // 有效點對不足，提前終止（退化或地圖空）
            result.is_degenerate = true;
            break;
        }

        // 2. 求解 GN 線性系統: H δx = -b
        Eigen::Matrix<double, 6, 1> dx = H.ldlt().solve(-b);

        // 3. 更新位姿 T ← exp(δx^) * T
        result.pose = applyLieUpdate(result.pose, dx);

        H_final = H;
        n_valid_final = n_valid;

        // 4. 收斂判斷
        if (dx.norm() < params_.convergence_threshold) {
            result.is_converged = true;
            break;
        }
    }

    if (n_valid_final > 0) {
        result.is_degenerate = checkDegenerate(H_final, params_.degenerate_threshold);
        result.covariance = computeCovariance(H_final, n_valid_final);
        // fitness_score: 最後一次迭代的平均點到面殘差（平方）
        // 由呼叫者透過 queryNearestPoints 自行評估，此處設為 0
        result.fitness_score = 0.0;
    }

    return result;
}

Eigen::Isometry3d IkdTreeScanMatcher::applyLieUpdate(const Eigen::Isometry3d& T,
                                                     const Eigen::Matrix<double, 6, 1>& dx) {
    // 使用 BCH 一階近似: exp(δx^) ≈ I + δx^
    // 完整版應使用 Rodrigues 公式，此處用小角度近似
    Eigen::Vector3d dt = dx.head<3>();
    Eigen::Vector3d dphi = dx.tail<3>();

    // 旋轉增量 (Rodrigues)
    double angle = dphi.norm();
    Eigen::Matrix3d dR;
    if (angle < 1e-9) {
        dR = Eigen::Matrix3d::Identity();
    } else {
        Eigen::AngleAxisd aa(angle, dphi / angle);
        dR = aa.toRotationMatrix();
    }

    Eigen::Isometry3d dT = Eigen::Isometry3d::Identity();
    dT.linear() = dR;
    dT.translation() = dt;

    return dT * T;  // left perturbation
}

Eigen::Matrix<double, 6, 6> IkdTreeScanMatcher::computeCovariance(
    const Eigen::Matrix<double, 6, 6>& H, int /*n_valid_points*/) {
    // 協方差 ≈ H^{-1}（假設殘差單位 sigma = 1）
    Eigen::Matrix<double, 6, 6> cov;
    bool invertible = false;
    H.computeInverseWithCheck(cov, invertible);
    if (!invertible) {
        cov = Eigen::Matrix<double, 6, 6>::Identity() * 1e-2;
    }
    return cov;
}

bool IkdTreeScanMatcher::checkDegenerate(const Eigen::Matrix<double, 6, 6>& H,
                                         double degenerate_threshold) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(H);
    auto eigenvalues = solver.eigenvalues();
    double lambda_max = eigenvalues(5);
    double lambda_min = eigenvalues(0);
    if (lambda_max < 1e-9) return true;
    return (lambda_max / (lambda_min + 1e-12)) > degenerate_threshold;
}

}  // namespace lio_slam_shaw::scan_matcher
