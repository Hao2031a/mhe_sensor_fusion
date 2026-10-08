cmake_minimum_required(VERSION 3.16)
project(mhe_sensor_fusion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

add_executable(mhe_sensor_fusion_node src/mhe_sensor_fusion.cpp)
# Informational 9x9 numerical kernel timing, never hard-gated as ROS speed.
add_executable(benchmark_rank_aware_kernel benchmark/benchmark_rank_aware_kernel.cpp)
target_include_directories(benchmark_rank_aware_kernel PRIVATE include)
target_link_libraries(benchmark_rank_aware_kernel PRIVATE Eigen3::Eigen)

target_include_directories(mhe_sensor_fusion_node PRIVATE include)

ament_target_dependencies(mhe_sensor_fusion_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(mhe_sensor_fusion_node
  Ceres::ceres
  Eigen3::Eigen
  Threads::Threads
)

target_compile_options(mhe_sensor_fusion_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS mhe_sensor_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch benchmark
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_horizon_observability benchmark/test_horizon_observability.cpp)
  target_include_directories(test_horizon_observability PRIVATE include)
  target_compile_features(test_horizon_observability PRIVATE cxx_std_17)
  add_test(NAME test_horizon_observability COMMAND $<TARGET_FILE:test_horizon_observability>)
  add_executable(test_horizon_dense_reference benchmark/test_horizon_dense_reference.cpp)
  target_include_directories(test_horizon_dense_reference PRIVATE include)
  target_compile_features(test_horizon_dense_reference PRIVATE cxx_std_17)
  add_test(NAME test_horizon_dense_reference COMMAND $<TARGET_FILE:test_horizon_dense_reference>)
  add_executable(test_gyro_increment benchmark/test_gyro_increment.cpp)
  target_include_directories(test_gyro_increment PRIVATE include)
  target_compile_features(test_gyro_increment PRIVATE cxx_std_17)
  add_test(NAME test_gyro_increment COMMAND $<TARGET_FILE:test_gyro_increment>)
  add_executable(test_marginal_factor_selector benchmark/test_marginal_factor_selector.cpp)
  target_include_directories(test_marginal_factor_selector PRIVATE include)
  add_test(NAME test_marginal_factor_selector COMMAND $<TARGET_FILE:test_marginal_factor_selector>)
  add_executable(test_stability_guards benchmark/test_stability_guards.cpp)
  target_include_directories(test_stability_guards PRIVATE include)
  target_compile_features(test_stability_guards PRIVATE cxx_std_17)
  add_test(NAME test_stability_guards COMMAND $<TARGET_FILE:test_stability_guards>)
  add_executable(test_rt_budget benchmark/test_rt_budget.cpp)
  target_include_directories(test_rt_budget PRIVATE include)
  target_compile_features(test_rt_budget PRIVATE cxx_std_17)
  add_test(NAME test_rt_budget COMMAND $<TARGET_FILE:test_rt_budget>)
  add_executable(test_so2_yaw_math benchmark/test_so2_yaw_math.cpp)
  target_include_directories(test_so2_yaw_math PRIVATE include)
  add_test(NAME test_so2_yaw_math COMMAND $<TARGET_FILE:test_so2_yaw_math>)
  add_executable(test_analytic_factors benchmark/test_analytic_factors.cpp)
  target_include_directories(test_analytic_factors PRIVATE include)
  target_compile_features(test_analytic_factors PRIVATE cxx_std_17)
  add_test(NAME test_analytic_factors COMMAND $<TARGET_FILE:test_analytic_factors>)
  add_executable(test_ceres_so2_yaw benchmark/test_ceres_so2_yaw.cpp)
  target_include_directories(test_ceres_so2_yaw PRIVATE include)
  target_link_libraries(test_ceres_so2_yaw PRIVATE Ceres::ceres)
  add_test(NAME test_ceres_so2_yaw COMMAND $<TARGET_FILE:test_ceres_so2_yaw>)
  add_executable(test_ceres_analytic benchmark/test_ceres_analytic.cpp)
  target_include_directories(test_ceres_analytic PRIVATE include)
  target_link_libraries(test_ceres_analytic PRIVATE Ceres::ceres)
  add_test(NAME test_ceres_analytic COMMAND $<TARGET_FILE:test_ceres_analytic>)
  add_executable(test_incremental_ceres benchmark/test_incremental_ceres.cpp)
  target_link_libraries(test_incremental_ceres PRIVATE Ceres::ceres)
  add_test(NAME test_incremental_ceres COMMAND $<TARGET_FILE:test_incremental_ceres>)
  add_executable(test_block_schur benchmark/test_block_schur.cpp)
  target_include_directories(test_block_schur PRIVATE include)
  target_link_libraries(test_block_schur PRIVATE Eigen3::Eigen)
  add_test(NAME test_block_schur COMMAND $<TARGET_FILE:test_block_schur>)
  add_executable(test_rank_aware_prior benchmark/test_rank_aware_prior.cpp)
  target_include_directories(test_rank_aware_prior PRIVATE include)
  target_link_libraries(test_rank_aware_prior PRIVATE Eigen3::Eigen)
  add_test(NAME test_rank_aware_prior COMMAND $<TARGET_FILE:test_rank_aware_prior>)
  add_executable(test_rank_aware_schur benchmark/test_rank_aware_schur.cpp)
  target_include_directories(test_rank_aware_schur PRIVATE include)
  target_link_libraries(test_rank_aware_schur PRIVATE Eigen3::Eigen)
  add_test(NAME test_rank_aware_schur COMMAND $<TARGET_FILE:test_rank_aware_schur>)
  add_executable(benchmark_covariance_math_ab benchmark/benchmark_covariance_math_ab.cpp)
  target_include_directories(benchmark_covariance_math_ab PRIVATE include)
  target_link_libraries(benchmark_covariance_math_ab PRIVATE Eigen3::Eigen Threads::Threads)
  add_test(NAME benchmark_covariance_math_ab COMMAND $<TARGET_FILE:benchmark_covariance_math_ab>)
  add_executable(test_scaled_covariance benchmark/test_scaled_covariance.cpp)
  target_include_directories(test_scaled_covariance PRIVATE include)
  target_link_libraries(test_scaled_covariance PRIVATE Eigen3::Eigen Threads::Threads)
  add_test(NAME test_scaled_covariance COMMAND $<TARGET_FILE:test_scaled_covariance>)
  add_executable(test_se2_error_correction benchmark/test_se2_error_correction.cpp)
  target_include_directories(test_se2_error_correction PRIVATE include)
  add_test(NAME test_se2_error_correction COMMAND $<TARGET_FILE:test_se2_error_correction>)
  add_executable(test_fast_covariance benchmark/test_fast_covariance.cpp)
  target_include_directories(test_fast_covariance PRIVATE include)
  target_link_libraries(test_fast_covariance PRIVATE Threads::Threads Eigen3::Eigen)
  add_test(NAME test_fast_covariance COMMAND $<TARGET_FILE:test_fast_covariance>)
  add_executable(test_time_aligned_output benchmark/test_time_aligned_output.cpp)
  target_include_directories(test_time_aligned_output PRIVATE include)
  target_compile_features(test_time_aligned_output PRIVATE cxx_std_17)
  add_test(NAME test_time_aligned_output COMMAND $<TARGET_FILE:test_time_aligned_output>)
  add_executable(test_low_latency_output benchmark/test_low_latency_output.cpp)
  target_include_directories(test_low_latency_output PRIVATE include)
  target_compile_features(test_low_latency_output PRIVATE cxx_std_17)
  add_test(NAME test_low_latency_output COMMAND $<TARGET_FILE:test_low_latency_output>)
  add_executable(test_covariance_calibration benchmark/test_covariance_calibration.cpp)
  target_include_directories(test_covariance_calibration PRIVATE include)
  target_link_libraries(test_covariance_calibration PRIVATE Eigen3::Eigen)
  add_test(NAME test_covariance_calibration COMMAND $<TARGET_FILE:test_covariance_calibration>)
endif()

ament_package()
