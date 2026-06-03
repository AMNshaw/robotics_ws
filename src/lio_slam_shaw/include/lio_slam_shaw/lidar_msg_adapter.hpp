#ifndef LIO_SLAM_SHAW__LIDAR_MSG_ADAPTOR__I_LIDAR_MSG_ADAPTER_HPP_
#define LIO_SLAM_SHAW__LIDAR_MSG_ADAPTOR__I_LIDAR_MSG_ADAPTER_HPP_

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <builtin_interfaces/msg/time.hpp>
#include <cmath>
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

inline float maxPointRelativeTime(const core::PointCloudIRT& cloud) {
    float max_time = 0.0f;
    for (const auto& point : cloud.points) {
        if (std::isfinite(point.time)) {
            max_time = std::max(max_time, point.time);
        }
    }
    return max_time;
}

inline core::LidarData convertVelodyne(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    auto cloud = std::make_shared<pcl::PointCloud<core::PointXYZIRT>>();

    pcl::fromROSMsg(*msg, *cloud);

    core::Timestamp timestamp = rclcppTimeToChrono(msg->header.stamp);
    core::Timestamp time_start = timestamp;
    core::Timestamp time_end = timestamp;

    if (!cloud->empty()) {
        time_end = time_start + floatSecondsToChrono(maxPointRelativeTime(*cloud));
    }

    return core::LidarData(timestamp, time_start, time_end, cloud);
}

inline core::LidarData convertOuster(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    const auto& fields = msg->fields;
    const uint8_t* raw = msg->data.data();
    const uint32_t point_step = msg->point_step;
    const uint32_t n_points = msg->width * msg->height;

    // Find field offsets and datatypes
    int off_x = -1, off_y = -1, off_z = -1;
    int off_intensity = -1, off_reflectivity = -1;
    int off_ring = -1;
    int off_t = -1, off_time = -1;
    uint8_t dt_ring = 0, dt_t = 0, dt_intensity = 0;

    for (const auto& f : fields) {
        if (f.name == "x")
            off_x = f.offset;
        else if (f.name == "y")
            off_y = f.offset;
        else if (f.name == "z")
            off_z = f.offset;
        else if (f.name == "intensity") {
            off_intensity = f.offset;
            dt_intensity = f.datatype;
        } else if (f.name == "reflectivity") {
            off_reflectivity = f.offset;
            dt_intensity = f.datatype;
        } else if (f.name == "ring") {
            off_ring = f.offset;
            dt_ring = f.datatype;
        } else if (f.name == "t") {
            off_t = f.offset;
            dt_t = f.datatype;
        } else if (f.name == "time") {
            off_time = f.offset;
            dt_t = f.datatype;
        }
    }

    const int i_off = (off_intensity >= 0) ? off_intensity : off_reflectivity;
    const int t_off = (off_time >= 0) ? off_time : off_t;

    auto cloud = std::make_shared<pcl::PointCloud<core::PointXYZIRT>>();
    cloud->resize(n_points);

    for (uint32_t i = 0; i < n_points; ++i) {
        const uint8_t* p = raw + i * point_step;
        auto& pt = cloud->points[i];

        pt.x = *reinterpret_cast<const float*>(p + off_x);
        pt.y = *reinterpret_cast<const float*>(p + off_y);
        pt.z = *reinterpret_cast<const float*>(p + off_z);

        // intensity / reflectivity: handle uint16, uint32, or float32
        if (i_off >= 0) {
            if (dt_intensity == sensor_msgs::msg::PointField::FLOAT32)
                pt.intensity = *reinterpret_cast<const float*>(p + i_off);
            else if (dt_intensity == sensor_msgs::msg::PointField::UINT16)
                pt.intensity = static_cast<float>(*reinterpret_cast<const uint16_t*>(p + i_off));
            else if (dt_intensity == sensor_msgs::msg::PointField::UINT32)
                pt.intensity = static_cast<float>(*reinterpret_cast<const uint32_t*>(p + i_off));
            else
                pt.intensity = 0.0f;
        } else {
            pt.intensity = 0.0f;
        }

        // ring: uint8 or uint16
        if (off_ring >= 0) {
            if (dt_ring == sensor_msgs::msg::PointField::UINT8)
                pt.ring = static_cast<uint16_t>(*(p + off_ring));
            else if (dt_ring == sensor_msgs::msg::PointField::UINT16)
                pt.ring = *reinterpret_cast<const uint16_t*>(p + off_ring);
            else
                pt.ring = 0;
        } else {
            pt.ring = 0;
        }

        // time: uint32 (nanoseconds) or float32 (seconds)
        if (t_off >= 0) {
            if (dt_t == sensor_msgs::msg::PointField::FLOAT32)
                pt.time = *reinterpret_cast<const float*>(p + t_off);
            else if (dt_t == sensor_msgs::msg::PointField::UINT32)
                pt.time = static_cast<float>(*reinterpret_cast<const uint32_t*>(p + t_off)) * 1e-9f;
            else
                pt.time = 0.0f;
        } else {
            pt.time = 0.0f;
        }
    }

    cloud->width = n_points;
    cloud->height = 1;
    cloud->is_dense = false;

    core::Timestamp timestamp = rclcppTimeToChrono(msg->header.stamp);
    core::Timestamp time_start = timestamp;
    core::Timestamp time_end = timestamp;

    if (!cloud->empty()) {
        time_end = time_start + floatSecondsToChrono(maxPointRelativeTime(*cloud));
    }

    return core::LidarData(timestamp, time_start, time_end, cloud);
}

// Not include livox driver yet, use sensor_msgs::msg::PointCloud2 as input for now. --- IGNORE ---

inline core::LidarData convertLivox(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    (void)msg;  // To avoid unused parameter warning
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