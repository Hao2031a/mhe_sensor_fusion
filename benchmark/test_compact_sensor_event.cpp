#include "mhe_sensor_fusion/compact_sensor_event.hpp"
#include <cassert>
#include <type_traits>
using mhe_sensor_fusion::CompactImu;
using mhe_sensor_fusion::CompactOdom;
using mhe_sensor_fusion::CompactSensorType;
int main()
{
  static_assert(std::is_trivially_copyable_v<CompactOdom>);
  static_assert(std::is_trivially_copyable_v<CompactImu>);
  static_assert(sizeof(CompactOdom) + sizeof(CompactImu) <= 144,
                "Event payload should stay bounded and compact");
  CompactOdom odom{};
  CompactImu imu{};
  assert(odom.qw == 1.0 && imu.qw == 1.0);
  assert(imu.orientation_covariance_00 == -1.0);
  assert(CompactSensorType::ODOM != CompactSensorType::IMU);
}
