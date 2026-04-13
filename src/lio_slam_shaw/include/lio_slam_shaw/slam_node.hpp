#ifndef LIO_SLAM_SHAW__SLAM_NODE_HPP_
#define LIO_SLAM_SHAW__SLAM_NODE_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "lio_slam_shaw/core/frame.hpp"
#include "lio_slam_shaw/core/i_feature_extractor.hpp"
#include "lio_slam_shaw/core/i_imu_preintegrator.hpp"
#include "lio_slam_shaw/core/i_lidar_deskewer.hpp"
#include "lio_slam_shaw/core/i_map_optimizer.hpp"

namespace lio_slam_shaw {

core::Timestamp rosToCore(const rclcpp::Time& ros_time) {
    return core::Timestamp(std::chrono::nanoseconds(ros_time.nanoseconds()));
}

class SlamNode : public rclcpp::Node {
public:
    explicit SlamNode(const rclcpp::NodeOptions& options);
    ~SlamNode() = default;

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    std::unique_ptr<core::ILidarDeskewer> deskewer_;
    std::unique_ptr<core::IFeatureExtractor> extractor_;
    std::unique_ptr<core::IImuPreintegrator> preintegrator_;
    std::unique_ptr<core::IMapOptimizer> optimizer_;

    std::deque<core::ImuData> imu_buffer_;
    std::mutex imu_mtx_;

    uint64_t frame_id_count_ = 0;
    double last_lidar_time_ = -1.0;

    void publishRealtimeOdometry(const core::ImuData& latest_imu);
    std::vector<core::ImuData> getImuSegment(double start_time, double end_time);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_realtime_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_mapped_;
};

}  // namespace lio_slam_shaw

#endif  // LIO_SLAM_SHAW__SLAM_NODE_HPP_