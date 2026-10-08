#pragma once

#include <cstddef>
#include <vector>

namespace mhe_sensor_fusion::marginalization
{
// A chain MHE stores unary factors at a state and pairwise factors under
// the index of their newer endpoint. To eliminate [0, removed), the only
// relevant factors are the previous prior, unary factors at [0, removed),
// and pairwise factors with newer endpoint in [1, removed]. The unary factors
// of the first retained state MUST NOT enter the Schur prior: the main graph
// still contains them and adding them again would double-count evidence.
//
// Preconditions: every factor connects one state or adjacent state pair;
// entries.size() >= removed + 1; prior is an active residual ID.
template<class Id, class Entry>
std::vector<Id> prefixCandidates(
  Id prior, const std::vector<Entry> & entries, std::size_t removed)
{
  std::vector<Id> ids;
  if (!prior || removed == 0 || removed >= entries.size()) {return ids;}
  ids.reserve(1 + 6 * (removed + 1));
  ids.push_back(prior);
  for (std::size_t k = 0; k <= removed; ++k) {
    if (k < removed) {
      ids.insert(ids.end(), entries[k].local.begin(), entries[k].local.end());
    }
    ids.insert(ids.end(), entries[k].transition.begin(), entries[k].transition.end());
  }
  return ids;
}
}  // namespace mhe_sensor_fusion::marginalization
