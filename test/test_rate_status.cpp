#include "rate_status.hpp"

#include <gtest/gtest.h>

#include <optional>
#include <stdexcept>

using generic_cam::classify_rate;
using generic_cam::DebouncedLevel;
using generic_cam::Level;
using generic_cam::RateLimits;
using generic_cam::RateStatus;

namespace
{
const RateLimits kBounds{28.0, 32.0, 20.0, 40.0};
}

TEST(ClassifyRate, SortsRatesIntoBands)
{
  EXPECT_EQ(classify_rate(kBounds, 30.0), Level::Ok);
  EXPECT_EQ(classify_rate(kBounds, 25.0), Level::Warn);
  EXPECT_EQ(classify_rate(kBounds, 35.0), Level::Warn);
  EXPECT_EQ(classify_rate(kBounds, 28.0), Level::Warn);
  EXPECT_EQ(classify_rate(kBounds, 19.0), Level::Error);
  EXPECT_EQ(classify_rate(kBounds, 41.0), Level::Error);
  EXPECT_EQ(classify_rate(kBounds, 0.0), Level::Error);
}

TEST(DebouncedLevel, StartsStaleAndNeedsRepeatsToWorsen)
{
  DebouncedLevel machine(3, false, false);
  EXPECT_EQ(machine.level(), Level::Stale);
  machine.push(Level::Warn);
  machine.push(Level::Warn);
  EXPECT_EQ(machine.level(), Level::Stale);
  EXPECT_EQ(machine.pending_count(), 2U);
  machine.push(Level::Warn);
  EXPECT_EQ(machine.level(), Level::Warn);
}

TEST(DebouncedLevel, AnotherObservationRestartsTheCount)
{
  DebouncedLevel machine(2, false, false);
  machine.push(Level::Ok);
  machine.push(Level::Ok);
  ASSERT_EQ(machine.level(), Level::Ok);
  machine.push(Level::Warn);
  machine.push(Level::Ok);
  machine.push(Level::Warn);
  EXPECT_EQ(machine.level(), Level::Ok);
}

TEST(DebouncedLevel, ImmediateErrorSkipsTheCount)
{
  DebouncedLevel machine(5, true, false);
  machine.push(Level::Error);
  EXPECT_EQ(machine.level(), Level::Error);
}

TEST(DebouncedLevel, ImmediateRelaxImprovesAtOnce)
{
  DebouncedLevel machine(1, false, true);
  machine.push(Level::Error);
  DebouncedLevel slower(4, true, true);
  slower.push(Level::Error);
  ASSERT_EQ(slower.level(), Level::Error);
  slower.push(Level::Ok);
  EXPECT_EQ(slower.level(), Level::Ok);
}

TEST(RateStatus, NoFramesStayStaleUntilARateArrives)
{
  RateStatus status(kBounds, 1, false, true);
  EXPECT_EQ(status.update(std::nullopt), Level::Stale);
  EXPECT_EQ(status.update(30.0), Level::Ok);
  EXPECT_EQ(status.update(0.0), Level::Error);
}

TEST(RateStatus, RejectsWarnRangeInsideOkRange)
{
  EXPECT_THROW(
    RateStatus(RateLimits{28.0, 32.0, 29.0, 40.0}, 1, false, true), std::invalid_argument);
  EXPECT_THROW(
    RateStatus(RateLimits{28.0, 32.0, 20.0, 31.0}, 1, false, true), std::invalid_argument);
}
