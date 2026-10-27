cmake_minimum_required(VERSION 3.12)
project(lio_slam_shaw)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree search registration)
find_package(GTSAM REQUIRED)
find_package(OpenMP REQUIRED)

set(THIRDPARTY_INCLUDE_DIRS
    SYSTEM ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${OpenMP_INCLUDE_DIRS}
)

include_directories(
    include
    ${THIRDPARTY_INCLUDE_DIRS}
    ThirdParty/ikd_tree
    ThirdParty/scancontext
)

add_library(imu_preintegrator
  src/imu_preintegrator/gtsam_imu_preintegrator.cpp
)
target_link_libraries(imu_preintegrator
  gtsam
  OpenMP::OpenMP_CXX
)

add_library(scan_preprocessor
  src/scan_preprocessor/imu_deskew_preprocessor.cpp
)
target_link_libraries(scan_preprocessor
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

add_library(map_builder
  src/map_builder/ikd_tree_local_map_builder.cpp
  src/map_builder/keyframe_global_map_builder.cpp
  ThirdParty/ikd_tree/ikd_tree.cpp
)
target_link_libraries(map_builder
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)
target_compile_options(map_builder PRIVATE -w)

add_library(scan_matcher
  src/scan_matcher/ikd_tree_scan_matcher.cpp
)
target_link_libraries(scan_matcher
  map_builder
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

add_library(feature_extractor
  src/feature_extractor/passthrough_feature_extractor.cpp
)
target_link_libraries(feature_extractor
  ${PCL_LIBRARIES}
)

add_library(odometry_estimator
  src/odometry_estimator/fast_lio_odometry.cpp
)
target_link_libraries(odometry_estimator
  map_builder
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

add_library(lio_initializer
  src/initializer/sfm_lio_initializer.cpp
)
target_link_libraries(lio_initializer
  scan_matcher
  map_builder
  ${PCL_LIBRARIES}
)

add_library(map_optimizer
  src/map_optimizer/gtsam_map_optimizer.cpp
)
target_link_libraries(map_optimizer
  gtsam
  OpenMP::OpenMP_CXX
)

add_library(loop_closure_detector
  src/loop_closure_detector/icp_loop_closure_detector.cpp
  src/loop_closure_detector/scan_context_manager.cpp
  src/loop_closure_detector/scan_context_loop_closure_detector.cpp
)
target_link_libraries(loop_closure_detector
  scan_matcher
  map_builder
  ${PCL_LIBRARIES}
)

add_library(core
  src/core/frontend.cpp
  src/core/backend.cpp
  src/core/slam_processor.cpp
)
target_link_libraries(core
  odometry_estimator
  scan_preprocessor
  feature_extractor
  map_builder
  map_optimizer
  loop_closure_detector
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # find_package(ament_cmake_gtest REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  # ament_add_gtest(test_sfm_lio_initializer
  #   test/test_sfm_lio_initializer.cpp
  # )
  # target_link_libraries(test_sfm_lio_initializer
  #   lio_initializer
  # )
  # target_include_directories(test_sfm_lio_initializer PRIVATE
  #   ${CMAKE_CURRENT_SOURCE_DIR}/include
  # )
endif()

ament_package()
