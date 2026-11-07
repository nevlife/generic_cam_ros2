#pragma once

#include <time.h>

#include <cstdint>

namespace generic_cam
{

inline int64_t read_clock_ns(clockid_t clock)
{
  timespec t{};
  clock_gettime(clock, &t);
  return static_cast<int64_t>(t.tv_sec) * 1000000000 + t.tv_nsec;
}

// CLOCK_REALTIME minus CLOCK_MONOTONIC. The realtime read sits between two monotonic reads and
// the pair with the shortest gap is kept, so a preemption between reads cannot skew the result.
inline int64_t realtime_minus_monotonic_ns()
{
  int64_t best_gap = INT64_MAX;
  int64_t offset = 0;
  for (int i = 0; i < 3; ++i) {
    const int64_t before = read_clock_ns(CLOCK_MONOTONIC);
    const int64_t realtime = read_clock_ns(CLOCK_REALTIME);
    const int64_t after = read_clock_ns(CLOCK_MONOTONIC);
    if (after - before < best_gap) {
      best_gap = after - before;
      offset = realtime - (before + (after - before) / 2);
    }
  }
  return offset;
}

}  // namespace generic_cam
