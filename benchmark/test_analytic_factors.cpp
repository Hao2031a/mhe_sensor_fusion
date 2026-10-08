#include "mhe_sensor_fusion/analytic_factors.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <random>
#include <stdexcept>

using namespace mhe_fusion::analytic;
static int checks = 0;
void checkNear(double actual, double expected, double tolerance, const char * what)
{
  ++checks;
  if (!std::isfinite(actual) || !std::isfinite(expected) ||
      std::abs(actual - expected) > tolerance * (1.0 + std::abs(expected))) {
    std::fprintf(stderr, "%s actual=%.17g expected=%.17g diff=%.17g\n",
      what, actual, expected, actual - expected);
    throw std::runtime_error(what);
  }
}

int main()
{
  std::mt19937_64 rng(0xC0FFEE);
  std::uniform_real_distribution<double> pos(-9.0, 9.0);
  std::uniform_real_distribution<double> slip(-0.4, 1.5);
  std::uniform_real_distribution<double> log_sigma(-5.0, 1.0);
  std::uniform_real_distribution<double> log_dt(-3.30103, -1.0);
  for (int trial = 0; trial < 1200; ++trial) {
    std::array<double, 9> a{}, b{}, sigma{};
    for (int i = 0; i < 9; ++i) {
      a[i] = pos(rng);
      b[i] = a[i] + 0.2 * pos(rng);
      sigma[i] = std::pow(10.0, log_sigma(rng));
    }
    a[kSl] = slip(rng); a[kSr] = slip(rng);
    b[kSl] = slip(rng); b[kSr] = slip(rng);
    const double dt = std::pow(10.0, log_dt(rng));
    ProcessKernel process(dt, sigma);
    double r[9], ja[81], jb[81];
    if (!process.evaluate(a.data(), b.data(), r, ja, jb)) {
      throw std::runtime_error("process rejected valid input");
    }
    // Independently re-derive the reference AutoDiff process residual.
    const double vavg = (a[kV] + b[kV]) / 2.0;
    const double wavg = (a[kW] + b[kW]) / 2.0;
    const double yaw = a[kYaw] + dt * wavg / 2.0;
    std::array<double,9> expected{};
    expected[kX] = (b[kX] - (a[kX] + dt * vavg * std::cos(yaw))) / sigma[kX];
    expected[kY] = (b[kY] - (a[kY] + dt * vavg * std::sin(yaw))) / sigma[kY];
    expected[kYaw] = (b[kYaw] - (a[kYaw] + dt * wavg)) / sigma[kYaw];
    for (int i = kV; i < 9; ++i) {
      expected[i] = (b[i] - a[i]) / sigma[i];
    }
    for (int i = 0; i < 9; ++i) {checkNear(r[i], expected[i], 2e-12, "process residual");}
    for (int side = 0; side < 2; ++side) {
      for (int var = 0; var < 9; ++var) {
        auto aa = a; auto bb = b;
        const double h = 1e-5 * std::max(1.0, std::abs(side ? b[var] : a[var]));
        (side ? bb[var] : aa[var]) += h;
        double rp[9]{};
        process.evaluate(aa.data(), bb.data(), rp, nullptr, nullptr);
        (side ? bb[var] : aa[var]) -= 2.0 * h;
        double rm[9]{};
        process.evaluate(aa.data(), bb.data(), rm, nullptr, nullptr);
        for (int row = 0; row < 9; ++row) {
          checkNear(side ? jb[9 * row + var] : ja[9 * row + var],
                    (rp[row] - rm[row]) / (2.0 * h), 1e-5,
                    "process Jacobian finite difference");
        }
      }
    }
    // Cases: one Jacobian requested, residual only, and both Jacobians.
    double one[81]{}, residue_only[9]{};
    process.evaluate(a.data(), b.data(), residue_only, one, nullptr);
    for (int j = 0; j < 81; ++j) {checkNear(one[j], ja[j], 1e-12, "partial process jac");}
    process.evaluate(a.data(), b.data(), residue_only, nullptr, one);
    for (int j = 0; j < 81; ++j) {checkNear(one[j], jb[j], 1e-12, "partial process jac1");}

    const double half_track = 0.05 + 0.2 * std::abs(pos(rng));
    const double sig_left = sigma[kV], sig_right = sigma[kW];
    const double left = pos(rng), right = pos(rng);
    WheelKernel wheel(left, right, 2.0 * half_track, sig_left, sig_right);
    double wr[2], wj[18];
    wheel.evaluate(a.data(), wr, wj);
    checkNear(wr[0], ((1.0 + a[kSl]) * (a[kV] - half_track * a[kW]) - left) / sig_left,
              2e-12, "wheel residual left");
    checkNear(wr[1], ((1.0 + a[kSr]) * (a[kV] + half_track * a[kW]) - right) / sig_right,
              2e-12, "wheel residual right");
    for (int var = 0; var < 9; ++var) {
      auto aa = a;
      const double h = 1e-5 * std::max(1.0, std::abs(a[var]));
      aa[var] += h;
      double rp[2]; wheel.evaluate(aa.data(), rp, nullptr);
      aa[var] -= 2*h;
      double rm[2]; wheel.evaluate(aa.data(), rm, nullptr);
      for (int row = 0; row < 2; ++row) {
        checkNear(wj[row*9 + var], (rp[row]-rm[row])/(2*h), 1e-5, "wheel Jacobian");
      }
    }
    double wr_only[2]; wheel.evaluate(a.data(), wr_only, nullptr);
    checkNear(wr_only[0], wr[0], 1e-12, "wheel residual only left");
    checkNear(wr_only[1], wr[1], 1e-12, "wheel residual only right");
  }
  std::printf("PASS %d deterministic residual/Jacobian checks across 1200 randomized horizons\n", checks);
}
