#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>

namespace generic_cam
{

// Values match diagnostic_msgs/DiagnosticStatus, so a smaller value is a healthier state.
enum class Level : uint8_t { Ok = 0, Warn = 1, Error = 2, Stale = 3 };

struct RateLimits
{
  double ok_min;
  double ok_max;
  double warn_min;
  double warn_max;
};

inline Level classify_rate(const RateLimits & limits, double rate_hz)
{
  if (rate_hz > limits.ok_min && rate_hz < limits.ok_max) {
    return Level::Ok;
  }
  const bool in_warn = rate_hz >= limits.warn_min && rate_hz <= limits.warn_max;
  return in_warn ? Level::Warn : Level::Error;
}

// Holds a level until the same new level is seen `repeats` times in a row.
class DebouncedLevel
{
public:
  DebouncedLevel(std::size_t repeats, bool error_at_once, bool recovery_at_once)
  : repeats_(repeats), error_at_once_(error_at_once), recovery_at_once_(recovery_at_once)
  {
  }

  Level push(Level seen)
  {
    pending_count_ = seen == pending_ ? pending_count_ + 1 : 1;
    pending_ = seen;
    const bool settled = pending_count_ >= repeats_;
    const bool urgent = error_at_once_ && seen == Level::Error;
    const bool recovered = recovery_at_once_ && seen < level_;
    if (seen != level_ && (settled || urgent || recovered)) {
      level_ = seen;
    }
    return level_;
  }

  Level level() const { return level_; }
  std::size_t pending_count() const { return pending_count_; }

private:
  std::size_t repeats_;
  bool error_at_once_;
  bool recovery_at_once_;
  Level level_{Level::Stale};
  Level pending_{Level::Stale};
  std::size_t pending_count_{0};
};

class RateStatus
{
public:
  RateStatus(
    const RateLimits & limits, std::size_t repeats, bool error_at_once, bool recovery_at_once)
  : limits_(limits), debounced_(repeats, error_at_once, recovery_at_once)
  {
    if (limits.warn_min >= limits.ok_min || limits.warn_max <= limits.ok_max) {
      throw std::invalid_argument("the warn range must enclose the ok range");
    }
  }

  // No value means that no rate could be measured yet.
  Level update(const std::optional<double> & rate_hz)
  {
    return rate_hz ? debounced_.push(classify_rate(limits_, *rate_hz)) : debounced_.level();
  }

private:
  RateLimits limits_;
  DebouncedLevel debounced_;
};

}  // namespace generic_cam
