#ifndef LIO_SLAM_SHAW__LOOP_CLOSURE_DETECTOR__I_LOOP_CLOSURE_DETECTOR_FACTORY_HPP_
#define LIO_SLAM_SHAW__LOOP_CLOSURE_DETECTOR__I_LOOP_CLOSURE_DETECTOR_FACTORY_HPP_

#include "lio_slam_shaw/core/i_loop_closure_detector.hpp"

namespace lio_slam_shaw::loop_closure_detector {

enum class LoopClosureDetectorType { SCAN_CONTEST };

class ILoopClosureDetectorFactory {
public:
    virtual ~ILoopClosureDetectorFactory() = default;
    virtual core::ILoopClosureDetector::SharedPtr create(LoopClosureDetectorType type) const = 0;
};

}  // namespace lio_slam_shaw::loop_closure_detector

#endif  // LIO_SLAM_SHAW__LOOP_CLOSURE_DETECTOR__I_LOOP_CLOSURE_DETECTOR_FACTORY_HPP_
