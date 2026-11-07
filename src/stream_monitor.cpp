#include "stream_monitor.hpp"

#include <algorithm>
#include <cstring>

namespace generic_cam
{

namespace
{

constexpr std::size_t kHashStride = 256;

uint64_t mix(uint64_t hash, uint64_t value)
{
  return (hash ^ value) * 0x100000001B3ULL;
}

}  // namespace

uint64_t content_hash(const uint8_t * data, std::size_t size)
{
  uint64_t hash = mix(0xCBF29CE484222325ULL, size);
  for (std::size_t at = 0; at + sizeof(uint64_t) <= size; at += kHashStride) {
    uint64_t word = 0;
    std::memcpy(&word, data + at, sizeof(word));
    hash = mix(hash, word);
  }
  return hash;
}

StreamMonitor::StreamMonitor(uint32_t frozen_frame_limit) : frozen_frame_limit_(frozen_frame_limit)
{
}

void StreamMonitor::on_frame(const FrameObservation & frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++window_.frames;
  window_.error_flags += frame.error_flag ? 1 : 0;
  if (!window_first_ns_) {
    window_first_ns_ = frame.capture_ns;
  }
  window_last_ns_ = frame.capture_ns;
  if (last_) {
    const int32_t step = static_cast<int32_t>(frame.sequence - last_->sequence);
    if (step <= 0 || frame.capture_ns <= last_->capture_ns) {
      ++window_.order_errors;
    } else if (step > 1) {
      ++window_.sequence_gaps;
      window_.missed_frames += static_cast<uint64_t>(step - 1);
      missed_frames_total_ += static_cast<uint64_t>(step - 1);
    }
    window_.identical_frames =
      frame.content_hash == last_->content_hash ? window_.identical_frames + 1 : 0;
  }
  last_ = frame;
}

void StreamMonitor::on_invalid_frame()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++window_.invalid_frames;
}

void StreamMonitor::on_dropped_publish()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++window_.dropped_publishes;
}

void StreamMonitor::on_published(int64_t capture_to_publish_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const double ms = static_cast<double>(capture_to_publish_ns) * 1e-6;
  ++window_published_;
  window_latency_sum_ms_ += ms;
  window_.latency_max_ms = std::max(window_.latency_max_ms, ms);
}

StreamReport StreamMonitor::collect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  StreamReport report = window_;
  report.missed_frames_total = missed_frames_total_;
  report.frozen = window_.identical_frames >= frozen_frame_limit_;
  if (window_published_ > 0) {
    report.latency_mean_ms = window_latency_sum_ms_ / static_cast<double>(window_published_);
  }
  if (window_.frames >= 2 && *window_last_ns_ > *window_first_ns_) {
    report.rate_hz = static_cast<double>(window_.frames - 1) * 1e9 /
                     static_cast<double>(*window_last_ns_ - *window_first_ns_);
    seen_rate_ = true;
  } else if (seen_rate_) {
    report.rate_hz = 0.0;
  }

  const uint32_t identical = window_.identical_frames;
  window_ = StreamReport{};
  window_.identical_frames = identical;
  window_first_ns_.reset();
  window_last_ns_.reset();
  window_published_ = 0;
  window_latency_sum_ms_ = 0.0;
  return report;
}

}  // namespace generic_cam
