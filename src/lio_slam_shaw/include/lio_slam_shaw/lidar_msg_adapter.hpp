#ifndef LIO_SLAM_SHAW__LIDAR_MSG_ADAPTOR__I_LIDAR_MSG_ADAPTER_HPP_
#define LIO_SLAM_SHAW__LIDAR_MSG_ADAPTOR__I_LIDAR_MSG_ADAPTER_HPP_

#include <pcl_conversions/pcl_conversions.h>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::lidar_msg_adaptor {

inline core::Timestamp rclcppTimeToChrono(const builtin_interfaces::msg::Time& stamp) {
    auto duration = std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec);
    return core::Timestamp(duration);
}

inline std::chrono::nanoseconds floatSecondsToChrono(float seconds) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<float>(seconds));
}

inline core::LidarData convertVelodyne(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    auto cloud = std::make_shared<pcl::PointCloud<core::PointXYZIRT>>();

    pcl::fromROSMsg(*msg, *cloud);

    core::Timestamp timestamp = rclcppTimeToChrono(msg->header.stamp);
    core::Timestamp time_start = timestamp;
    core::Timestamp time_end = timestamp;

    if (!cloud->empty()) {
        time_end = time_start + floatSecondsToChrono(cloud->points.back().time);
    }

    return core::LidarData(timestamp, time_start, time_end, cloud);
}

inline core::LidarData convertOuster(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    sensor_msgs::msg::PointCloud2 msg_modified = *msg;

    for (auto& field : msg_modified.fields) {
        if (field.name == "t") {
            field.name = "time";
            break;
        }
    }

    for (auto& field : msg_modified.fields) {
        if (field.name == "reflectivity") {
            field.name = "intensity";
            break;
        }
    }

    auto cloud = std::make_shared<pcl::PointCloud<core::PointXYZIRT>>();
    pcl::fromROSMsg(msg_modified, *cloud);

    core::Timestamp timestamp = rclcppTimeToChrono(msg->header.stamp);
    core::Timestamp time_start = timestamp;
    core::Timestamp time_end = timestamp;

    if (!cloud->empty()) {
        time_end = time_start + floatSecondsToChrono(cloud->points.back().time);
    }

    return core::LidarData(timestamp, time_start, time_end, cloud);
}

// Not include livox driver yet, use sensor_msgs::msg::PointCloud2 as input for now. --- IGNORE ---

inline core::LidarData convertLivox(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    return core::LidarData();
}

// inline core::LidarData convertLivox(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg) {
//     auto cloud = std::make_shared<pcl::PointCloud<core::PointXYZIRT>>();
//     cloud->reserve(msg->points.size());

//     // Livox 的時間基底 (timebase) 單位是奈秒
//     core::Timestamp timestamp = floatSecondsToChrono(msg->timebase * 1e-9);
//     core::Timestamp time_start = timestamp;
//     core::Timestamp time_end = timestamp;

//     for (const auto& pt : msg->points) {
//         core::PointXYZIRT p;
//         p.x = pt.x;
//         p.y = pt.y;
//         p.z = pt.z;
//         p.intensity = pt.reflectivity;
//         p.ring = pt.line;
//         p.time = pt.offset_time * 1e-9f;  // 奈秒轉秒

//         cloud->push_back(p);
//     }

//     if (!cloud->empty()) {
//         time_end = time_start + floatSecondsToChrono(cloud->points.back().time);
//     }

//     return core::LidarData(timestamp, time_start, time_end, cloud);
// }

}  // namespace lio_slam_shaw::lidar_msg_adaptor

#endif  // LIO_SLAM_SHAW__LIDAR_MSG_ADAPTOR__I_LIDAR_MSG_ADAPTER_HPP_