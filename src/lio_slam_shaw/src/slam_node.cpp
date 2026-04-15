#include "lio_slam_shaw/slam_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
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

    bool use_tf_extrinsic = declare_parameter("use_tf_extrinsic", false);
    std::string tracking_frame_id = "";
    std::string lidar_frame_id = "";
    std::string imu_frame_id = "";
    if (use_tf_extrinsic) {
        tracking_frame_id = declare_parameter("tracking_frame_id", "base_link");
        lidar_frame_id = declare_parameter("lidar_frame_id", "lidar_link");
        imu_frame_id = declare_parameter("imu_frame_id", "imu_link");
    }

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("scan", 10);

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

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    slam_processor_->registerOdometryCallback(
        [this](const core::NavState& odom_state) { publishOdometry(odom_state); });
    slam_processor_->registerVisualizationCallback(
        [this](const core::VisualizationData& viz_data) { publishVisualization(viz_data); });

    RCLCPP_INFO(get_logger(), "LIO-SLAM-Shaw Node initialized.");
}

SlamNode::~SlamNode() {
    RCLCPP_INFO(get_logger(), "Shutting down LIO-SLAM-Shaw Node...");
    rclcpp::shutdown();
    RCLCPP_INFO(get_logger(), "LIO-SLAM-Shaw Node shut down.");
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

void SlamNode::publishOdometry(const core::NavState& odom_state) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = coreToRos(odom_state.timestamp);
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = odom_state.pose.translation().x();
    odom_msg.pose.pose.position.y = odom_state.pose.translation().y();
    odom_msg.pose.pose.position.z = odom_state.pose.translation().z();
    Eigen::Quaterniond q(odom_state.pose.rotation());
    geometry_msgs::msg::Quaternion quat;
    quat.x = q.x();
    quat.y = q.y();
    quat.z = q.z();
    quat.w = q.w();
    odom_msg.pose.pose.orientation = quat;
    odom_msg.twist.twist.linear.x = odom_state.linear_vel.x();
    odom_msg.twist.twist.linear.y = odom_state.linear_vel.y();
    odom_msg.twist.twist.linear.z = odom_state.linear_vel.z();
    odom_msg.twist.twist.angular.x = odom_state.angular_vel.x();
    odom_msg.twist.twist.angular.y = odom_state.angular_vel.y();
    odom_msg.twist.twist.angular.z = odom_state.angular_vel.z();

    odom_publisher_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = odom_msg.header.stamp;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = odom_state.pose.translation().x();
    tf_msg.transform.translation.y = odom_state.pose.translation().y();
    tf_msg.transform.translation.z = odom_state.pose.translation().z();
    tf_msg.transform.rotation = quat;

    tf_broadcaster_->sendTransform(tf_msg);
}

void SlamNode::publishVisualization(const core::VisualizationData& viz_data) {
    auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = coreToRos(viz_data.timestamp);
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "odom";
    tf_msg.transform.translation.x = viz_data.T_map_odom.translation().x();
    tf_msg.transform.translation.y = viz_data.T_map_odom.translation().y();
    tf_msg.transform.translation.z = viz_data.T_map_odom.translation().z();
    Eigen::Quaterniond q(viz_data.T_map_odom.rotation());
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);

    cloud_msg->header.stamp = coreToRos(viz_data.timestamp);
    cloud_msg->header.frame_id = "odom";
    pcl::toROSMsg(*(viz_data.scan), *cloud_msg);
    cloud_publisher_->publish(*cloud_msg);
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