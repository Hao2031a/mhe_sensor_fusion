#pragma once
// Numerically stable SO(2) angular difference and exact constant-twist SE(2)
// integration. Functions are compatible with Ceres Jet via argument-dependent
// lookup of sin/cos/atan2. Nonlinear wrapping is only smooth locally, away
// from the antipodal (+/- pi) cut.
#include <cmath>

namespace mhe_fusion::yaw_math {
constexpr double kPi = 3.14159265358979323846264338327950288;

template<class T>
inline T wrapDifference(const T & difference)
{
  // Local-chart fast path: the derivative is exactly 1 and Ceres need not
  // evaluate transcendental functions on ordinary small yaw innovations.
  if (difference > T(-kPi) && difference < T(kPi)) {return difference;}
  using std::atan2;
  using std::sin;
  using std::cos;
  return atan2(sin(difference), cos(difference));
}

// sinc(z) = sin(z)/z, with an even Taylor polynomial near 0 to avoid 0/0.
template<class T>
inline T sinc(const T & z)
{
  const T z2 = z * z;
  // Jet supports comparisons by scalar part; branch is smooth at the threshold
  // to within the order of the series remainder.
  if (z > T(-1e-3) && z < T(1e-3)) {
    return T(1.0) + z2 * (T(-1.0 / 6.0) +
      z2 * (T(1.0 / 120.0) + z2 * T(-1.0 / 5040.0)));
  }
  using std::sin;
  return sin(z) / z;
}

inline double sincDerivative(double z)
{
  const double z2 = z * z;
  if (std::abs(z) < 1e-3) {
    return z * (-1.0 / 3.0 + z2 * (1.0 / 30.0 - z2 / 840.0));
  }
  return (z * std::cos(z) - std::sin(z)) / z2;
}

// Exact planar integration for constant forward body speed and angular speed
// over dt: displacement=v*dt*sinc(dtheta/2)*R(theta+dtheta/2)*[1,0].
// Works at w=0 without singular division by angular speed.
template<class T>
inline void integrateArc(const T & yaw, const T & v, const T & w,
  const T & dt, T & delta_x, T & delta_y)
{
  const T half = T(0.5) * w * dt;
  const T move = v * dt * sinc(half);
  using std::cos;
  using std::sin;
  delta_x = move * cos(yaw + half);
  delta_y = move * sin(yaw + half);
}

struct ArcDerivatives {
  double dx{0.0};
  double dy{0.0};
  double dx_dyaw{0.0};
  double dy_dyaw{0.0};
  double dx_dv{0.0};
  double dy_dv{0.0};
  double dx_dw{0.0};
  double dy_dw{0.0};
};

inline ArcDerivatives arcWithDerivatives(double yaw, double v, double w, double dt)
{
  const double half = 0.5 * w * dt;
  const double s = sinc(half);
  const double ds = sincDerivative(half);
  const double c = std::cos(yaw + half);
  const double si = std::sin(yaw + half);
  ArcDerivatives a;
  a.dx = v * dt * s * c;
  a.dy = v * dt * s * si;
  a.dx_dyaw = -a.dy;
  a.dy_dyaw = a.dx;
  a.dx_dv = dt * s * c;
  a.dy_dv = dt * s * si;
  a.dx_dw = 0.5 * v * dt * dt * (ds * c - s * si);
  a.dy_dw = 0.5 * v * dt * dt * (ds * si + s * c);
  return a;
}
}  // namespace mhe_fusion::yaw_math
