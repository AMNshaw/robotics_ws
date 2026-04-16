#include "lio_slam_shaw/scan_matcher/ikd_tree_scan_matcher.hpp"

#include <omp.h>

#include <Eigen/Dense>
#include <cmath>

namespace lio_slam_shaw::scan_matcher {

IkdTreeScanMatcher::IkdTreeScanMatcher(core::IMapBuilder::SharedPtr map_builder,
                                       const IkdTreeScanMatcherParams& params)
    : map_builder_(std::move(map_builder)), params_(params) {
    T_base_lidar_ = params_.T_base_lidar;
}

void IkdTreeScanMatcher::setLidarExtrinsics(const Eigen::Isometry3d& T_base_lidar) {
    T_base_lidar_ = T_base_lidar;
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
        const Eigen::Isometry3d T_map_lidar = result.pose * T_base_lidar_;
        const Eigen::Matrix3d R_map_lidar = T_map_lidar.linear();
        const Eigen::Vector3d t_map_lidar = T_map_lidar.translation();

        std::vector<NearestPlaneResult> nearest_planes;
        nearest_planes.resize(cloud->size());
#pragma omp parallel for schedule(static)
        for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
            const auto& pt = (*cloud)[i];
            core::PointXYZIRT query_map_pt = pt;
            query_map_pt.x =
                static_cast<float>(R_map_lidar(0, 0) * pt.x + R_map_lidar(0, 1) * pt.y +
                                   R_map_lidar(0, 2) * pt.z + t_map_lidar.x());
            query_map_pt.y =
                static_cast<float>(R_map_lidar(1, 0) * pt.x + R_map_lidar(1, 1) * pt.y +
                                   R_map_lidar(1, 2) * pt.z + t_map_lidar.y());
            query_map_pt.z =
                static_cast<float>(R_map_lidar(2, 0) * pt.x + R_map_lidar(2, 1) * pt.y +
                                   R_map_lidar(2, 2) * pt.z + t_map_lidar.z());
            std::vector<core::PointXYZIRT> neighbors;
            std::vector<float> distances;
            if (!map_builder_->searchKNearestPoints(query_map_pt, params_.k_neighbors,
                                                    params_.max_search_dist, neighbors, distances))
                continue;

            if (static_cast<int>(neighbors.size()) >= params_.min_plane_points) {
                nearest_planes[i] = fitPlane(
                    neighbors, Eigen::Vector3d(query_map_pt.x, query_map_pt.y, query_map_pt.z));
            } else {
                nearest_planes[i].valid = false;
            }
        }

        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
        int n_valid = 0;

        for (const auto& n : nearest_planes) {
            if (!n.valid) continue;

            Eigen::Vector3d normal = n.normal;
            Eigen::Vector3d dir_to_sensor = t_map_lidar - n.point_in_map;

            if (normal.dot(dir_to_sensor) < 0) {
                normal = -normal;
            }

            double r = normal.dot(n.point_in_map - n.centroid);

            Eigen::Matrix<double, 6, 1> J;
            J.head<3>() = normal;
            Eigen::Vector3d pt_diff = n.point_in_map - t_map_lidar;
            J.tail<3>() = pt_diff.cross(normal);

            H += J * J.transpose();
            b += J * r;
            ++n_valid;
        }

        if (n_valid < params_.min_valid_points) {
            result.is_degenerate = true;
            break;
        }

        Eigen::Matrix<double, 6, 1> dx = H.ldlt().solve(-b);

        if (dx.hasNaN()) {
            std::cerr << "[Warning] Solver produced NaN, aborting iteration." << std::endl;
            break;
        }

        result.pose = applyLieUpdate(result.pose, dx, t_map_lidar);

        H_final = H;
        n_valid_final = n_valid;

        std::cerr << "Iteration " << iter << ": dx norm = " << dx.norm()
                  << ", valid points = " << n_valid << std::endl;

        if (dx.norm() < params_.convergence_threshold) {
            result.is_converged = true;
            break;
        }
    }

    if (n_valid_final > 0) {
        result.is_degenerate = checkDegenerate(H_final, params_.degenerate_threshold);
        result.covariance = computeCovariance(H_final, n_valid_final);
        result.fitness_score = 0.0;
    }

    return result;
}

NearestPlaneResult IkdTreeScanMatcher::fitPlane(
    const std::vector<lio_slam_shaw::core::PointXYZIRT>& neighbors,
    const Eigen::Vector3d& query_point_in_map) const {
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& p : neighbors) centroid += Eigen::Vector3d(p.x, p.y, p.z);
    centroid /= static_cast<double>(neighbors.size());

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : neighbors) {
        Eigen::Vector3d dp = Eigen::Vector3d(p.x, p.y, p.z) - centroid;
        cov += dp * dp.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    const auto& eigenvalues = solver.eigenvalues();

    if (eigenvalues(0) > params_.plane_valid_threshold * eigenvalues(2)) {
        return {false, {}, {}, {}};
    }

    Eigen::Vector3d normal = solver.eigenvectors().col(0);

    return NearestPlaneResult{
        /*valid=*/true,
        /*point_in_map=*/query_point_in_map,
        /*normal=*/normal,
        /*centroid=*/centroid,
    };
}

Eigen::Isometry3d IkdTreeScanMatcher::applyLieUpdate(
    const Eigen::Isometry3d& T, const Eigen::Matrix<double, 6, 1>& dx,
    const Eigen::Vector3d& rot_center) {  // 👈 記得加參數
    Eigen::Vector3d dt = dx.head<3>();
    Eigen::Vector3d dphi = dx.tail<3>();

    double angle = dphi.norm();
    Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();  // 預設為單位矩陣
    if (angle > 1e-9) {
        Eigen::AngleAxisd aa(angle, dphi / angle);
        dR = aa.toRotationMatrix();
    }

    Eigen::Isometry3d dT_new = Eigen::Isometry3d::Identity();

    // 🌟 核心修正：讓旋轉繞著 rot_center (LiDAR 中心) 轉，而不是世界原點！
    dT_new.linear() = dR * T.linear();
    dT_new.translation() = dR * (T.translation() - rot_center) + dt + rot_center;

    return dT_new;
}

Eigen::Matrix<double, 6, 6> IkdTreeScanMatcher::computeCovariance(
    const Eigen::Matrix<double, 6, 6>& H, int /*n_valid_points*/) {
    Eigen::Matrix<double, 6, 6> cov;
    Eigen::FullPivLU<Eigen::Matrix<double, 6, 6>> lu(H);
    if (lu.isInvertible()) {
        cov = lu.inverse();
    } else {
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
