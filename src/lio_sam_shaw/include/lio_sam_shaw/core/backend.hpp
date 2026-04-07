#ifndef LIO_SAM_SHAW__CORE__BACKEND_HPP_
#define LIO_SAM_SHAW__CORE__BACKEND_HPP_

#include "lio_sam_shaw/core/frame.hpp"
#include "lio_sam_shaw/core/i_feature_extractor.hpp"
#include "lio_sam_shaw/core/i_imu_preintegrator.hpp"
#include "lio_sam_shaw/core/i_scan_matcher.hpp"
#include "lio_sam_shaw/core/i_scan_preprocessor.hpp"

namespace lio_sam_shaw::core {

class BackEnd {
   public:
    using SharedPtr = std::shared_ptr<BackEnd>;
    using ConstSharedPtr = std::shared_ptr<const BackEnd>;

    BackEnd() = default;
    ~BackEnd() = default;

    void processData();

   private:
};
}  // namespace lio_sam_shaw::core
#endif  // LIO_SAM_SHAW__CORE__BACKEND_HPP_