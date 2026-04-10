#include "lio_slam_shaw/factory/imu_preintegrator_factory.hpp"

#include "lio_slam_shaw/imu_preintegrator/gtsam_imu_preintegrator.hpp"

namespace lio_slam_shaw::factory {

core::IImuPreintegrator::SharedPtr ImuPreintegratorFactory::create(rclcpp::Node::SharedPtr node) {
    std::string type = node->declare_parameter<std::string>("imu_preintegrator_type", "gtsam");

    GtsamImuPreintegratorParams params;
    params.gravity = node->declare_parameter<double>("imu.gravity", params.gravity);
    params.imu_acc_noise = node->declare_parameter<double>("imu.acc_noise", params.imu_acc_noise);
    params.imu_gyr_noise = node->declare_parameter<double>("imu.gyr_noise", params.imu_gyr_noise);
    params.imu_acc_bias_noise =
        node->declare_parameter<double>("imu.acc_bias_noise", params.imu_acc_bias_noise);
    params.imu_gyr_bias_noise =
        node->declare_parameter<double>("imu.gyr_bias_noise", params.imu_gyr_bias_noise);
    params.T_base_imu_trans = node->declare_parameter<std::vector<double>>("imu.T_base_imu_trans",
                                                                           params.T_base_imu_trans);
    params.T_base_imu_rot =
        node->declare_parameter<std::vector<double>>("imu.T_base_imu_rot", params.T_base_imu_rot);

    if (type != "gtsam") {
        RCLCPP_WARN(node->get_logger(),
                    "Unknown imu_preintegrator_type '%s', falling back to gtsam.", type.c_str());
    }
    return std::make_shared<GtsamImuPreintegrator>(params);
}

}  // namespace lio_slam_shaw::factory
