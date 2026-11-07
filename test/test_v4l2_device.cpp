#include "v4l2_device.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

using generic_cam::CaptureFormat;
using generic_cam::control_param_name;
using generic_cam::PixelFormat;
using generic_cam::V4l2Device;

TEST(ControlParamName, MatchesV4l2CtlNames)
{
  EXPECT_EQ(control_param_name("Brightness"), "brightness");
  EXPECT_EQ(control_param_name("Exposure Time, Absolute"), "exposure_time_absolute");
  EXPECT_EQ(control_param_name("White Balance, Automatic"), "white_balance_automatic");
  EXPECT_EQ(control_param_name("Focus, Automatic Continuous"), "focus_automatic_continuous");
  EXPECT_EQ(control_param_name("Gain (dB)"), "gain_db");
}

TEST(V4l2Device, MissingDeviceFailsAtConstruction)
{
  const CaptureFormat format{PixelFormat::Mjpeg, 640, 480, 30};
  EXPECT_THROW(V4l2Device("/dev/generic_cam_no_such_device", format), std::runtime_error);
}
