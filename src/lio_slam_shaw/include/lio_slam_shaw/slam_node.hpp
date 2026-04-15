#ifndef LIO_SLAM_SHAW__SLAM_NODE_HPP_
#define LIO_SLAM_SHAW__SLAM_NODE_HPP_

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include <deque>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "lio_slam_shaw/core/sensor_data_types.hpp"
#include "lio_slam_shaw/core/slam_processor.hpp"

namespace lio_slam_shaw {

inline core::Timestamp rosToCore(const rclcpp::Time& ros_time) {
    return core::Timestamp(std::chrono::nanoseconds(ros_time.nanoseconds()));
}

inline rclcpp::Time coreToRos(const core::Timestamp& t) {
    return rclcpp::Time(
        std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count(),
        RCL_SYSTEM_TIME);
}

class SlamNode : public rclcpp::Node {
public:
    explicit SlamNode(const rclcpp::NodeOptions& options);
    ~SlamNode();

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    void velodyneLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void ousterLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void livoxLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void publishOdometry(const core::NavState& odom_state);

    void publishVisualization(const core::VisualizationData& viz_data);

    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_publisher_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> cloud_publisher_;

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_subscription_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> velodyne_subscription_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> ouster_subscription_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> livox_subscription_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::shared_ptr<core::SlamProcessor> slam_processor_;
};

}  // namespace lio_slam_shaw

#endif  // LIO_SLAM_SHAW__SLAM_NODE_HPP_