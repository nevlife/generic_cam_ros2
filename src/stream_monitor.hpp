#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>

namespace generic_cam
{

struct FrameObservation
{
  int64_t capture_ns;
  uint32_t sequence;
  bool error_flag;
  uint64_t content_hash;
};

// Counts since the previous collect(), except the fields marked total.
struct StreamReport
{
  std::optional<double> rate_hz;
  uint64_t frames{0};
  uint64_t sequence_gaps{0};
  uint64_t missed_frames{0};
  uint64_t order_errors{0};
  uint64_t error_flags{0};
  uint64_t invalid_frames{0};
  uint64_t dropped_publishes{0};
  uint64_t missed_frames_total{0};
  uint32_t identical_frames{0};
  bool frozen{false};
  double latency_mean_ms{0.0};
  double latency_max_ms{0.0};
};

// Samples the buffer so that a full-size frame costs microseconds.
uint64_t content_hash(const uint8_t * data, std::size_t size);

// Written by a capture thread and read by the diagnostic timer, hence the mutex.
class StreamMonitor
{
public:
  explicit StreamMonitor(uint32_t frozen_frame_limit);

  void on_frame(const FrameObservation & frame);
  void on_invalid_frame();
  void on_dropped_publish();
  void on_published(int64_t capture_to_publish_ns);
  StreamReport collect();

private:
  std::mutex mutex_;
  uint32_t frozen_frame_limit_;
  StreamReport window_;
  std::optional<FrameObservation> last_;
  std::optional<int64_t> window_first_ns_;
  std::optional<int64_t> window_last_ns_;
  uint64_t window_published_{0};
  double window_latency_sum_ms_{0.0};
  uint64_t missed_frames_total_{0};
  bool seen_rate_{false};
};

}  // namespace generic_cam
