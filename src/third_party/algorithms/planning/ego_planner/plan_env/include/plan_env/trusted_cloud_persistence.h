#ifndef PLAN_ENV_TRUSTED_CLOUD_PERSISTENCE_H
#define PLAN_ENV_TRUSTED_CLOUD_PERSISTENCE_H

#include <cstddef>
#include <cstdint>
#include <limits>
#include <algorithm>
#include <vector>

namespace plan_env
{

inline void clearTrustedCloudStorage(
  std::vector<char> & occupied,
  std::vector<std::uint16_t> & age)
{
  std::fill(occupied.begin(), occupied.end(), 0);
  std::fill(age.begin(), age.end(), 0);
}

// The semantic cloud contains obstacle returns only.  Therefore a point that
// is absent from a later frame is not evidence that the corresponding static
// obstacle is free.  Keep an occupied voxel for a bounded number of trusted
// frames and let the caller clear it immediately when the cloud is invalid or
// explicitly empty.
inline bool ageTrustedCloudVoxels(
  std::vector<char> & occupied,
  std::vector<std::uint16_t> & age,
  const std::size_t max_frames)
{
  if (occupied.size() != age.size() || max_frames == 0 ||
    max_frames > static_cast<std::size_t>(std::numeric_limits<std::uint16_t>::max()))
  {
    return false;
  }

  const auto limit = static_cast<std::uint16_t>(max_frames);
  for (std::size_t i = 0; i < occupied.size(); ++i) {
    if (occupied[i] == 0) {
      age[i] = 0;
      continue;
    }

    if (age[i] + 1U < limit) {
      ++age[i];
    } else {
      // The voxel was not observed in max_frames consecutive trusted frames.
      // Expire it on the frame that reaches the configured boundary, before
      // processing that frame's marks.  A current re-observation below can
      // immediately restore it with age zero.
      occupied[i] = 0;
      age[i] = 0;
    }
  }
  return true;
}

inline void markTrustedCloudVoxel(
  std::vector<char> & occupied,
  std::vector<std::uint16_t> & age,
  const std::size_t index)
{
  if (index >= occupied.size() || index >= age.size()) {
    return;
  }
  occupied[index] = 1;
  age[index] = 0;
}

inline void clearTrustedCloudVoxel(
  std::vector<char> & occupied,
  std::vector<std::uint16_t> & age,
  const std::size_t index)
{
  if (index >= occupied.size() || index >= age.size()) {
    return;
  }
  occupied[index] = 0;
  age[index] = 0;
}

}  // namespace plan_env

#endif  // PLAN_ENV_TRUSTED_CLOUD_PERSISTENCE_H
