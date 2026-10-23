#include "lio_slam_shaw/loop_closure_detector/scan_context_manager.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>

namespace lio_slam_shaw {

ScanContextManager::ScanContextManager(const ScanContextParams& params) : params_(params) {}

// ---------------------------------------------------------------------------

ScDescriptor ScanContextManager::getDescriptor(const core::PointCloudIRTConstPtr& cloud) const {
    const float NO_POINT = -1000.0f;
    Eigen::MatrixXf desc = NO_POINT * Eigen::MatrixXf::Ones(params_.num_ring, params_.num_sector);

    for (const auto& p : *cloud) {
        float z_lifted = p.z + static_cast<float>(params_.lidar_height);
        float range = std::sqrt(p.x * p.x + p.y * p.y);
        if (range > static_cast<float>(params_.max_radius)) continue;

        float angle_deg = std::atan2(p.y, p.x) * (180.0f / M_PI);
        if (angle_deg < 0.0f) angle_deg += 360.0f;

        int ring_idx = std::max(
            1, std::min(params_.num_ring, static_cast<int>(std::ceil((range / params_.max_radius) *
                                                                     params_.num_ring))));
        int sect_idx = std::max(
            1, std::min(params_.num_sector,
                        static_cast<int>(std::ceil((angle_deg / 360.0f) * params_.num_sector))));

        float& cell = desc(ring_idx - 1, sect_idx - 1);
        if (cell < z_lifted) cell = z_lifted;
    }

    for (int r = 0; r < desc.rows(); ++r)
        for (int c = 0; c < desc.cols(); ++c)
            if (desc(r, c) == NO_POINT) desc(r, c) = 0.0f;

    const Eigen::VectorXf rk = makeRingKey(desc);
    return ScDescriptor{desc, std::vector<float>(rk.data(), rk.data() + rk.size())};
}

// ---------------------------------------------------------------------------

std::optional<std::pair<uint64_t, float>> ScanContextManager::findClosest(
    const ScDescriptor& query, const std::unordered_map<uint64_t, ScDescriptor>& candidates) const {
    if (candidates.empty()) return std::nullopt;

    // Stage 1: ring-key L2 distance to find top-K candidates
    std::vector<std::pair<float, uint64_t>> ring_dists;
    ring_dists.reserve(candidates.size());
    for (const auto& [id, desc] : candidates) {
        float dist_sq = 0.0f;
        for (size_t i = 0; i < query.ring_key.size() && i < desc.ring_key.size(); ++i) {
            float d = query.ring_key[i] - desc.ring_key[i];
            dist_sq += d * d;
        }
        ring_dists.emplace_back(dist_sq, id);
    }

    const int k = std::min(params_.num_candidates, static_cast<int>(ring_dists.size()));
    std::partial_sort(ring_dists.begin(), ring_dists.begin() + k, ring_dists.end());

    // Stage 2: pairwise SC distance on top-K
    int best_shift = 0;
    uint64_t best_id = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (int i = 0; i < k; ++i) {
        const uint64_t cand_id = ring_dists[i].second;
        const auto it = candidates.find(cand_id);

        // Normal direction
        auto [dist, shift] = distanceBtnSC(query.mat, it->second.mat);
        if (dist < min_dist) {
            min_dist = dist;
            best_shift = shift;
            best_id = cand_id;
        }

        // SC++ reverse: mirror sectors to handle opposite-direction revisit
        if (params_.enable_reverse_search) {
            const Eigen::MatrixXf flipped = it->second.mat.rowwise().reverse();
            auto [dist_r, shift_r] = distanceBtnSC(query.mat, flipped);
            if (dist_r < min_dist) {
                min_dist = dist_r;
                best_shift = params_.num_sector - shift_r;  // compensate flip
                best_id = cand_id;
            }
        }
    }

    if (min_dist >= params_.sc_dist_threshold) {
        std::clog << "[SC] best_dist=" << min_dist << " >= threshold=" << params_.sc_dist_threshold
                  << " (rejected, best_id=" << best_id << ")\n";
        return std::nullopt;
    }

    const float yaw_diff_rad =
        static_cast<float>(best_shift) * static_cast<float>(2.0 * M_PI / params_.num_sector);
    std::clog << "[SC] MATCHED id=" << best_id << " dist=" << min_dist << " yaw=" << yaw_diff_rad
              << "rad\n";
    return std::make_pair(best_id, yaw_diff_rad);
}

// ---------------------------------------------------------------------------

Eigen::VectorXf ScanContextManager::makeRingKey(const Eigen::MatrixXf& desc) const {
    return desc.rowwise().mean();
}

Eigen::RowVectorXf ScanContextManager::makeSectorKey(const Eigen::MatrixXf& desc) const {
    return desc.colwise().mean();
}

Eigen::MatrixXf ScanContextManager::circshift(const Eigen::MatrixXf& mat, int n) {
    if (n == 0) return mat;
    const int cols = mat.cols();
    Eigen::MatrixXf shifted = Eigen::MatrixXf::Zero(mat.rows(), cols);
    for (int c = 0; c < cols; ++c) {
        shifted.col((c + n) % cols) = mat.col(c);
    }
    return shifted;
}

double ScanContextManager::distDirectSC(const Eigen::MatrixXf& sc1,
                                        const Eigen::MatrixXf& sc2) const {
    int num_eff = 0;
    double sum_sim = 0.0;
    for (int c = 0; c < sc1.cols(); ++c) {
        const Eigen::VectorXf col1 = sc1.col(c);
        const Eigen::VectorXf col2 = sc2.col(c);
        const float n1 = col1.norm();
        const float n2 = col2.norm();
        if (n1 < 1e-6f || n2 < 1e-6f) continue;
        sum_sim += static_cast<double>(col1.dot(col2)) / (static_cast<double>(n1) * n2);
        ++num_eff;
    }
    return num_eff > 0 ? 1.0 - sum_sim / num_eff : 1.0;
}

int ScanContextManager::fastAlignUsingVkey(const Eigen::RowVectorXf& vk1,
                                           const Eigen::RowVectorXf& vk2) const {
    int best_shift = 0;
    float min_diff = std::numeric_limits<float>::max();
    const int cols = static_cast<int>(vk1.cols());
    for (int s = 0; s < cols; ++s) {
        float diff_sq = 0.0f;
        for (int c = 0; c < cols; ++c) {
            float d = vk1(c) - vk2((c + s) % cols);
            diff_sq += d * d;
        }
        if (diff_sq < min_diff) {
            min_diff = diff_sq;
            best_shift = s;
        }
    }
    return best_shift;
}

std::pair<double, int> ScanContextManager::distanceBtnSC(const Eigen::MatrixXf& sc1,
                                                         const Eigen::MatrixXf& sc2) const {
    const int argmin_shift = fastAlignUsingVkey(makeSectorKey(sc1), makeSectorKey(sc2));

    const int half_range = static_cast<int>(std::round(0.5 * params_.search_ratio * sc1.cols()));
    std::vector<int> shift_space = {argmin_shift};
    for (int ii = 1; ii <= half_range; ++ii) {
        shift_space.push_back((argmin_shift + ii + sc1.cols()) % sc1.cols());
        shift_space.push_back((argmin_shift - ii + sc1.cols()) % sc1.cols());
    }

    int best_shift = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (int s : shift_space) {
        const double d = distDirectSC(sc1, circshift(sc2, s));
        if (d < min_dist) {
            min_dist = d;
            best_shift = s;
        }
    }
    return {min_dist, best_shift};
}

}  // namespace lio_slam_shaw
