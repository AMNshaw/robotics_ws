#ifndef LIO_SLAM_SHAW__SLAM_NODE_HPP_
#define LIO_SLAM_SHAW__SLAM_NODE_HPP_

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

#include "lio_slam_shaw/core/sensor_data_types.hpp"
#include "lio_slam_shaw/core/slam_processor.hpp"

namespace lio_slam_shaw {

core::Timestamp rosToCore(const rclcpp::Time& ros_time) {
    return core::Timestamp(std::chrono::nanoseconds(ros_time.nanoseconds()));
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

    void visualizationThread();

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_subscription_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> velodyne_subscription_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> ouster_subscription_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> livox_subscription_;

    std::shared_ptr<core::SlamProcessor> slam_processor_;

    std::deque<core::VisualizationData> viz_queue_;
    std::mutex viz_mutex_;
    std::condition_variable viz_cv_;
    std::thread viz_thread_;
};

}  // namespace lio_slam_shaw

#endif  // LIO_SLAM_SHAW__SLAM_NODE_HPP_