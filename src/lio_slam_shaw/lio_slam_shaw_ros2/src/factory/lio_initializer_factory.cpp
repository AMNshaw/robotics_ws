#include "lio_slam_shaw_ros2/factory/lio_initializer_factory.hpp"

#include "lio_slam_shaw/initializer/sfm_lio_initializer.hpp"

namespace lio_slam_shaw_ros2::factory {

core::ILioInitializer::SharedPtr RosLioInitializerFactory::create(
    const Eigen::Isometry3d& T_base_lidar, const Eigen::Isometry3d& T_base_imu) const {
    lio_slam_shaw::scan_matcher::Config scan_matcher_params;
    lio_slam_shaw::map_builder::IkdTreeLocalMapBuilderParams map_builder_params;
    lio_slam_shaw::initializer::Config sfm_init_params;
    Eigen::Isometry3d T_imu_lidar = T_base_imu.inverse() * T_base_lidar;

    // Load params from ROS (if provided)

    scan_matcher_params.T_base_lidar = T_base_lidar;
    scan_matcher_params.k_neighbors =
        node_->declare_parameter<int>("initializer.scan_matcher.k_neighbors", 5);
    scan_matcher_params.max_iterations =
        node_->declare_parameter<int>("initializer.scan_matcher.max_iterations", 30);
    scan_matcher_params.convergence_threshold =
        node_->declare_parameter<double>("initializer.scan_matcher.convergence_threshold", 1e-5);
    scan_matcher_params.min_valid_points =
        node_->declare_parameter<int>("initializer.scan_matcher.min_valid_points", 50);
    scan_matcher_params.degenerate_threshold =
        node_->declare_parameter<double>("initializer.scan_matcher.degenerate_threshold", 100.0);
    scan_matcher_params.max_search_dist =
        node_->declare_parameter<double>("initializer.scan_matcher.max_search_dist", 2.0);
    scan_matcher_params.min_plane_points =
        node_->declare_parameter<int>("initializer.scan_matcher.min_plane_points", 5);
    scan_matcher_params.plane_valid_threshold =
        node_->declare_parameter<double>("initializer.scan_matcher.plane_valid_threshold", 0.1);
    scan_matcher_params.rot_regularization_sigma =
        node_->declare_parameter<double>("initializer.scan_matcher.rot_regularization_sigma", 0.0);

    map_builder_params.ikd_delete_param = static_cast<float>(
        node_->declare_parameter<double>("initializer.map_builder.ikd_delete_param", 0.5));
    map_builder_params.ikd_balance_param = static_cast<float>(
        node_->declare_parameter<double>("initializer.map_builder.ikd_balance_param", 0.6));
    map_builder_params.ikd_downsample_size = static_cast<float>(
        node_->declare_parameter<double>("initializer.map_builder.ikd_downsample_size", 0.3));

    sfm_init_params.min_init_scans =
        node_->declare_parameter("initializer.sfm.min_init_scans", sfm_init_params.min_init_scans);
    sfm_init_params.align_gravity =
        node_->declare_parameter("initializer.sfm.align_gravity", sfm_init_params.align_gravity);
    sfm_init_params.voxel_leaf_size = node_->declare_parameter("initializer.sfm.voxel_leaf_size",
                                                               sfm_init_params.voxel_leaf_size);
    sfm_init_params.acc_noise =
        node_->declare_parameter("initializer.sfm.acc_noise", sfm_init_params.acc_noise);
    sfm_init_params.gyr_noise =
        node_->declare_parameter("initializer.sfm.gyr_noise", sfm_init_params.gyr_noise);

    return std::make_shared<lio_slam_shaw::initializer::SfmLioInitializer>(
        scan_matcher_params, map_builder_params, T_imu_lidar, sfm_init_params);
}

}  // namespace lio_slam_shaw_ros2::factory