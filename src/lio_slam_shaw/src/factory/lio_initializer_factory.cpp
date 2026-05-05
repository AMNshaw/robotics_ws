#include "lio_slam_shaw/factory/lio_initializer_factory.hpp"

#include "lio_slam_shaw/initializer/sfm_lio_initializer.hpp"

namespace lio_slam_shaw::factory {

core::ILioInitializer::SharedPtr LioInitializerFactory::create(
    rclcpp::Node* node, const Eigen::Isometry3d& T_base_lidar,
    const Eigen::Isometry3d& T_base_imu) {
    // For now we only have one implementation, but this factory allows us to
    // easily add more in the future and select via ROS params.
    scan_matcher::IkdTreeScanMatcherParams scan_matcher_params;
    map_builder::IkdTreeMapBuilderParams map_builder_params;
    initializer::SfmLioInitializerParams sfm_init_params;
    Eigen::Isometry3d T_imu_lidar = T_base_imu.inverse() * T_base_lidar;

    // Load params from ROS (if provided)

    scan_matcher_params.T_base_lidar = T_base_lidar;
    scan_matcher_params.k_neighbors =
        node->declare_parameter<int>("initializer.scan_matcher.k_neighbors", 5);
    scan_matcher_params.max_iterations =
        node->declare_parameter<int>("initializer.scan_matcher.max_iterations", 30);
    scan_matcher_params.convergence_threshold =
        node->declare_parameter<double>("initializer.scan_matcher.convergence_threshold", 1e-5);
    scan_matcher_params.min_valid_points =
        node->declare_parameter<int>("initializer.scan_matcher.min_valid_points", 50);
    scan_matcher_params.degenerate_threshold =
        node->declare_parameter<double>("initializer.scan_matcher.degenerate_threshold", 100.0);
    scan_matcher_params.max_search_dist =
        node->declare_parameter<double>("initializer.scan_matcher.max_search_dist", 2.0);
    scan_matcher_params.min_plane_points =
        node->declare_parameter<int>("initializer.scan_matcher.min_plane_points", 5);
    scan_matcher_params.plane_valid_threshold =
        node->declare_parameter<double>("initializer.scan_matcher.plane_valid_threshold", 0.1);
    scan_matcher_params.rot_regularization_sigma =
        node->declare_parameter<double>("initializer.scan_matcher.rot_regularization_sigma", 0.0);

    map_builder_params.keyframe_distance_threshold =
        node->declare_parameter<double>("initializer.map_builder.keyframe_distance_threshold", 1.0);
    map_builder_params.keyframe_angle_threshold =
        node->declare_parameter<double>("initializer.map_builder.keyframe_angle_threshold", 0.2);
    map_builder_params.ikd_delete_param = static_cast<float>(
        node->declare_parameter<double>("initializer.map_builder.ikd_delete_param", 0.5));
    map_builder_params.ikd_balance_param = static_cast<float>(
        node->declare_parameter<double>("initializer.map_builder.ikd_balance_param", 0.6));
    map_builder_params.ikd_downsample_size = static_cast<float>(
        node->declare_parameter<double>("initializer.map_builder.ikd_downsample_size", 0.3));

    sfm_init_params.min_init_scans =
        node->declare_parameter("initializer.sfm.min_init_scans", sfm_init_params.min_init_scans);
    sfm_init_params.align_gravity =
        node->declare_parameter("initializer.sfm.align_gravity", sfm_init_params.align_gravity);
    sfm_init_params.voxel_leaf_size =
        node->declare_parameter("initializer.sfm.voxel_leaf_size", sfm_init_params.voxel_leaf_size);
    sfm_init_params.acc_noise =
        node->declare_parameter("initializer.sfm.acc_noise", sfm_init_params.acc_noise);
    sfm_init_params.gyr_noise =
        node->declare_parameter("initializer.sfm.gyr_noise", sfm_init_params.gyr_noise);

    return std::make_shared<initializer::SfmLioInitializer>(scan_matcher_params, map_builder_params,
                                                            T_imu_lidar, sfm_init_params);
}

}  // namespace lio_slam_shaw::factory