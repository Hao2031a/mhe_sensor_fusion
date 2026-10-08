// ROS/Ceres-free exact-set regression for prefix factor selection.
#include "mhe_sensor_fusion/marginal_factor_selector.hpp"
#include <algorithm>
#include <stdexcept>
#include <cstdint>
#include <cstdio>
#include <set>
#include <vector>

struct Entry
{
  std::vector<int *> local;
  std::vector<int *> transition;
};

int main()
{
  int prior = 1;
  int count = 0;
  for (std::size_t n = 6; n <= 64; ++n) {
    std::vector<Entry> entries(n);
    std::vector<int> ids(n * 8, 0);
    for (std::size_t k = 0; k < n; ++k) {
      const int nlocal = 1 + static_cast<int>((k * 17 + n) % 5);
      for (int i = 0; i < nlocal; ++i) {
        entries[k].local.push_back(&ids[8 * k + static_cast<std::size_t>(i)]);
      }
      if (k) {
        for (int i = 0; i < 2; ++i) {
          entries[k].transition.push_back(&ids[8 * k + 5 + static_cast<std::size_t>(i)]);
        }
      }
    }
    for (std::size_t removed = 1; removed + 4 < n; ++removed) {
      auto selected = mhe_sensor_fusion::marginalization::prefixCandidates(
        &prior, entries, removed);
      std::set<int *> expected{&prior};
      for (std::size_t k = 0; k < n; ++k) {
        if (k < removed) expected.insert(entries[k].local.begin(), entries[k].local.end());
        if (k <= removed) expected.insert(entries[k].transition.begin(), entries[k].transition.end());
      }
      std::set<int *> actual(selected.begin(), selected.end());
      // Must remain an actual check in Release (-DNDEBUG) CI builds.
      if (actual != expected || actual.size() != selected.size()) {
        throw std::runtime_error("incorrect or repeated marginal factor IDs");
      }
      ++count;
    }
  }
  std::printf("PASS %d randomized-shape prefix factor-set checks\n", count);
  return 0;
}
