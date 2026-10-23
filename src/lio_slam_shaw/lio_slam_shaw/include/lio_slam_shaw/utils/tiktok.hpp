#ifndef LIO_SLAM_SHAW__UTILS__TIKTOK_HPP_
#define LIO_SLAM_SHAW__UTILS__TIKTOK_HPP_

#include <chrono>
#include <iostream>
#include <string>

namespace lio_slam_shaw::utils {

/// Lightweight scoped timer. Construct with a name, call tok() to print elapsed ms.
class TikTok {
public:
    explicit TikTok(const std::string& name)
        : name_(name), start_(std::chrono::steady_clock::now()) {}

    /// Print elapsed ms since construction and return the value.
    double tok() {
        auto now = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(now - start_).count();
        std::clog << "[" << name_ << "] " << ms << "ms\n";
        return ms;
    }

private:
    std::string name_;
    std::chrono::steady_clock::time_point start_;
};

}  // namespace lio_slam_shaw::utils

#endif  // LIO_SLAM_SHAW__UTILS__TIKTOK_HPP_
