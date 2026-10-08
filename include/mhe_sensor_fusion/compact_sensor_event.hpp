#pragma once

#include <cstdint>

namespace mhe_sensor_fusion
{
// Only the scalar fields needed by the estimator are copied out of ROS messages.
// The queue never owns or copies a full nav_msgs/Odometry or sensor_msgs/Imu.
enum class CompactSensorType : std::uint8_t { ODOM, IMU };

struct CompactOdom
{
  double px{0.0}, py{0.0};
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
  double v{0.0}, w{0.0};
};

struct CompactImu
{
  double gyro_z{0.0};
  double ax{0.0}, ay{0.0}, az{0.0};
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
  double orientation_covariance_00{-1.0};
};
}  // namespace mhe_sensor_fusion
