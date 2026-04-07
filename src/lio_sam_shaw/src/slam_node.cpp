#include "lio_sam_shaw/slam_node.hpp"

namespace lio_sam_shaw {

SlamNode::SlamNode(const rclcpp::NodeOptions& options) : Node("slam_node", options) {
    RCLCPP_INFO(this->get_logger(), "Initializing LIO-SAM-SHAW Node...");

    // --- 1. 使用 Factory 初始化元件 (依賴注入) ---
    // 這裡假設你有寫好對應的 Factory，並傳入 this (Node 指標) 來讀取 ROS 參數
    // deskewer_ = factory::LidarDeskewerFactory::create(this);
    // extractor_ = factory::FeatureExtractorFactory::create(this);
    // preintegrator_ = factory::ImuPreintegratorFactory::create(this);
    // optimizer_ = factory::MapOptimizerFactory::create(this);

    // --- 2. 訂閱者 (Subscribers) ---
    auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu_raw", imu_qos, std::bind(&SlamNode::imuCallback, this, std::placeholders::_1));

    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "points_raw", 10, std::bind(&SlamNode::lidarCallback, this, std::placeholders::_1));

    // --- 3. 發布者 (Publishers) ---
    pub_odom_realtime_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_preint", 10);
    pub_odom_mapped_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_mapped", 10);
}

void SlamNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // 1. 轉換為 Core 格式
    core::ImuData imu;
    imu.timestamp = rclcpp::Time(msg->header.stamp).seconds();
    imu.linear_acceleration << msg->linear_acceleration.x, msg->linear_acceleration.y,
        msg->linear_acceleration.z;
    imu.angular_velocity << msg->angular_velocity.x, msg->angular_velocity.y,
        msg->angular_velocity.z;

    // 2. 存入 Buffer (供 Lidar 執行序回溯)
    {
        std::lock_guard<std::mutex> lock(imu_mtx_);
        imu_buffer_.push_back(imu);

        // 定期修剪 Buffer (保留最近 5 秒即可)
        while (imu_buffer_.size() > 1 && (imu.timestamp - imu_buffer_.front().timestamp > 5.0)) {
            imu_buffer_.pop_front();
        }
    }

    // 3. 即時預測並發布高頻里程計 (200Hz+)
    if (preintegrator_) {
        preintegrator_->predict(imu);  // 執行單步積分
        publishRealtimeOdometry(imu);
    }
}

void SlamNode::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    double current_time = rclcpp::Time(msg->header.stamp).seconds();

    // 第一幀初始化
    if (last_lidar_time_ < 0) {
        last_lidar_time_ = current_time;
        return;
    }

    // --- 1. 建立影格 ---
    auto frame = core::LidarFrame::make_frame(frame_id_count_++, current_time);
    // 將 ROS 點雲轉成 PCL 存入 frame->raw_cloud (這裡省略轉換 code)

    // --- 2. 獲取同步的 IMU 切片 ---
    auto imu_segment = getImuSegment(last_lidar_time_, current_time);
    if (imu_segment.empty()) {
        RCLCPP_WARN(this->get_logger(), "No IMU data between lidar scans!");
        return;
    }

    // --- 3. 被動觸發核心 Pipeline ---

    // A. 預積分 (更新因子圖所需的 Delta 狀態)
    preintegrator_->integrateBatch(imu_segment);

    // B. 去畸變 (修正點雲幾何)
    deskewer_->deskew(frame, imu_segment, {});

    // C. 特徵提取 (Edge & Surf)
    extractor_->extract(frame);

    // D. 優化器 (因子圖更新)
    if (optimizer_->update(frame)) {
        // 優化成功，發布精確位姿
        // pub_odom_mapped_->publish(...);

        if (frame->is_keyframe) {
            core::NavState optimized_state{frame->timestamp, frame->pose, frame->vel};
            preintegrator_->reset(optimized_state, frame->bias);
        }
    }

    last_lidar_time_ = current_time;
}

std::vector<core::ImuData> SlamNode::getImuSegment(double start_time, double end_time) {
    std::vector<core::ImuData> segment;
    std::lock_guard<std::mutex> lock(imu_mtx_);

    if (imu_buffer_.empty()) return segment;

    // 使用二分搜尋優化檢索效能 O(log N)
    auto it_start =
        std::lower_bound(imu_buffer_.begin(), imu_buffer_.end(), start_time,
                         [](const core::ImuData& d, double t) { return d.timestamp < t; });

    auto it_end =
        std::lower_bound(it_start, imu_buffer_.end(), end_time,
                         [](const core::ImuData& d, double t) { return d.timestamp < t; });

    segment.assign(it_start, it_end);
    return segment;
}

void SlamNode::publishRealtimeOdometry(const core::ImuData& latest_imu) {
    // 從預積分器獲取最新的預測狀態 (P, V, R)
    core::NavState latest_state = preintegrator_->getLatestNavState();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = rclcpp::Time(static_cast<uint64_t>(latest_state.timestamp * 1e9));
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // 填入位姿與速度 (轉換 Eigen 到 ROS 格式)
    // odom.pose.pose = ...
    // odom.twist.twist = ...

    pub_odom_realtime_->publish(odom);
}

}  // namespace lio_sam_shaw