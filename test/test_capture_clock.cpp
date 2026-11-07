#include "capture_clock.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>

using generic_cam::read_clock_ns;
using generic_cam::realtime_minus_monotonic_ns;

TEST(CaptureClock, OffsetMapsMonotonicOntoRealtime)
{
  const int64_t offset = realtime_minus_monotonic_ns();
  const int64_t mapped = read_clock_ns(CLOCK_MONOTONIC) + offset;
  EXPECT_LT(std::abs(read_clock_ns(CLOCK_REALTIME) - mapped), 5000000);
}

TEST(CaptureClock, OffsetIsStableBetweenReads)
{
  EXPECT_LT(std::abs(realtime_minus_monotonic_ns() - realtime_minus_monotonic_ns()), 1000000);
}
