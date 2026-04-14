#include "lio_slam_shaw/slam_node.hpp"

#include <set>

#include "lio_slam_shaw/factory/slam_factory.hpp"
#include "lio_slam_shaw/lidar_msg_adapter.hpp"

namespace lio_slam_shaw {

SlamNode::SlamNode(const rclcpp::NodeOptions& options) : Node("lio_slam_shaw_node", options) {
    RCLCPP_INFO(get_logger(), "Initializing LIO-SLAM-Shaw Node...");

    slam_processor_ = factory::SlamFactory::create(shared_from_this());

    const std::string lidar_type = declare_parameter("lidar_type", "Velodyne");
    const std::string lidar_topic = declare_parameter("lidar_topic", "");
    if (lidar_topic.empty()) {
        RCLCPP_ERROR(get_logger(), "Lidar topic is not specified.");
        throw std::runtime_error("Lidar topic is not specified");
    }
    const std::string imu_topic = declare_parameter("imu_topic", "");
    if (imu_topic.empty()) {
        RCLCPP_ERROR(get_logger(), "Imu topic is not specified.");
        throw std::runtime_error("Imu topic is not specified");
    }

    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 100, std::bind(&SlamNode::imuCallback, this, std::placeholders::_1));

    if (lidar_type == "Velodyne") {
        velodyne_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic, 10,
            std::bind(&SlamNode::velodyneLidarCallback, this, std::placeholders::_1));
        ouster_subscription_ = nullptr;
        livox_subscription_ = nullptr;
    } else if (lidar_type == "Ouster") {
        ouster_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic, 10,
            std::bind(&SlamNode::ousterLidarCallback, this, std::placeholders::_1));
        velodyne_subscription_ = nullptr;
        livox_subscription_ = nullptr;
    } else if (lidar_type == "Livox") {
        livox_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic, 10, std::bind(&SlamNode::livoxLidarCallback, this, std::placeholders::_1));
        velodyne_subscription_ = nullptr;
        ouster_subscription_ = nullptr;
    } else {
        RCLCPP_ERROR(get_logger(),
                     "Unsupported Lidar Type: %s. Supported types are: Velodyne, Ouster, Livox.",
                     lidar_type.c_str());
        throw std::runtime_error("Invalid Lidar Type");
    }

    RCLCPP_INFO(get_logger(), "LIO-SLAM-Shaw Node initialized.");
}

void SlamNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    core::Timestamp timestamp = rosToCore(msg->header.stamp);

    core::ImuData imu_data;
    imu_data.timestamp = timestamp;
    imu_data.acc = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y,
                                   msg->linear_acceleration.z);
    imu_data.gyr =
        Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    slam_processor_->feedImu(imu_data);
}

void SlamNode::velodyneLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto lidar_data = lidar_msg_adaptor::convertVelodyne(msg);
    slam_processor_->feedLidar(lidar_data);
}

void SlamNode::ousterLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto lidar_data = lidar_msg_adaptor::convertOuster(msg);
    slam_processor_->feedLidar(lidar_data);
}

void SlamNode::livoxLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto lidar_data = lidar_msg_adaptor::convertLivox(msg);
    slam_processor_->feedLidar(lidar_data);
}

}  // namespace lio_slam_shaw

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lio_slam_shaw::SlamNode>(rclcpp::NodeOptions());
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->spin();
    rclcpp::shutdown();
    return 0;
}