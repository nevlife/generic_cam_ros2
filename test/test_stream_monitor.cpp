#include "stream_monitor.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

using generic_cam::content_hash;
using generic_cam::FrameObservation;
using generic_cam::StreamMonitor;

namespace
{

constexpr int64_t kPeriodNs = 33333333;

FrameObservation frame(uint32_t sequence, uint64_t hash = 0)
{
  return {
    static_cast<int64_t>(sequence) * kPeriodNs + 1000, sequence, false,
    hash == 0 ? sequence + 1U : hash};
}

}  // namespace

TEST(StreamMonitor, MeasuresRateOverTheWindow)
{
  StreamMonitor monitor(10);
  for (uint32_t i = 0; i < 31; ++i) {
    monitor.on_frame(frame(i));
  }
  const auto report = monitor.collect();
  ASSERT_TRUE(report.rate_hz.has_value());
  EXPECT_NEAR(*report.rate_hz, 30.0, 0.01);
  EXPECT_EQ(report.frames, 31U);
  EXPECT_EQ(report.sequence_gaps, 0U);
}

TEST(StreamMonitor, NoRateBeforeFramesAndZeroAfterTheyStop)
{
  StreamMonitor monitor(10);
  EXPECT_FALSE(monitor.collect().rate_hz.has_value());
  monitor.on_frame(frame(0));
  EXPECT_FALSE(monitor.collect().rate_hz.has_value());
  monitor.on_frame(frame(1));
  monitor.on_frame(frame(2));
  EXPECT_TRUE(monitor.collect().rate_hz.has_value());
  const auto report = monitor.collect();
  ASSERT_TRUE(report.rate_hz.has_value());
  EXPECT_DOUBLE_EQ(*report.rate_hz, 0.0);
}

TEST(StreamMonitor, CountsSequenceGaps)
{
  StreamMonitor monitor(10);
  monitor.on_frame(frame(5));
  monitor.on_frame(frame(6));
  monitor.on_frame(frame(9));
  auto report = monitor.collect();
  EXPECT_EQ(report.sequence_gaps, 1U);
  EXPECT_EQ(report.missed_frames, 2U);
  monitor.on_frame(frame(11));
  report = monitor.collect();
  EXPECT_EQ(report.missed_frames, 1U);
  EXPECT_EQ(report.missed_frames_total, 3U);
}

TEST(StreamMonitor, ReportsSequenceWrapAsNormal)
{
  StreamMonitor monitor(10);
  monitor.on_frame({1000, UINT32_MAX, false, 1});
  monitor.on_frame({2000, 0, false, 2});
  const auto report = monitor.collect();
  EXPECT_EQ(report.order_errors, 0U);
  EXPECT_EQ(report.sequence_gaps, 0U);
}

TEST(StreamMonitor, FlagsRepeatedSequenceAndBackwardTimestamp)
{
  StreamMonitor monitor(10);
  monitor.on_frame({2000, 3, false, 1});
  monitor.on_frame({3000, 3, false, 2});
  monitor.on_frame({1000, 4, false, 3});
  EXPECT_EQ(monitor.collect().order_errors, 2U);
}

TEST(StreamMonitor, DetectsFrozenImageAcrossWindowsAndClearsIt)
{
  StreamMonitor monitor(5);
  uint32_t sequence = 0;
  for (; sequence < 4; ++sequence) {
    monitor.on_frame(frame(sequence, 77));
  }
  EXPECT_FALSE(monitor.collect().frozen);
  for (; sequence < 7; ++sequence) {
    monitor.on_frame(frame(sequence, 77));
  }
  EXPECT_TRUE(monitor.collect().frozen);
  monitor.on_frame(frame(sequence, 78));
  EXPECT_FALSE(monitor.collect().frozen);
}

TEST(StreamMonitor, CountsErrorsDropsAndLatency)
{
  StreamMonitor monitor(10);
  monitor.on_frame({1000, 1, true, 1});
  monitor.on_invalid_frame();
  monitor.on_dropped_publish();
  monitor.on_published(2000000);
  monitor.on_published(4000000);
  auto report = monitor.collect();
  EXPECT_EQ(report.error_flags, 1U);
  EXPECT_EQ(report.invalid_frames, 1U);
  EXPECT_EQ(report.dropped_publishes, 1U);
  EXPECT_DOUBLE_EQ(report.latency_mean_ms, 3.0);
  EXPECT_DOUBLE_EQ(report.latency_max_ms, 4.0);
  report = monitor.collect();
  EXPECT_EQ(report.error_flags, 0U);
  EXPECT_DOUBLE_EQ(report.latency_max_ms, 0.0);
}

TEST(ContentHash, FollowsContentAndSize)
{
  std::vector<uint8_t> first(100000, 1);
  std::vector<uint8_t> second = first;
  EXPECT_EQ(content_hash(first.data(), first.size()), content_hash(second.data(), second.size()));
  second[2560] = 2;
  EXPECT_NE(content_hash(first.data(), first.size()), content_hash(second.data(), second.size()));
  EXPECT_NE(content_hash(first.data(), 90000), content_hash(first.data(), 90001));
}
