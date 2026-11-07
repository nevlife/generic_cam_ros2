#include "frame_decoder.hpp"

#include <gtest/gtest.h>
#include <jpeglib.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using generic_cam::CaptureFormat;
using generic_cam::make_decoder;
using generic_cam::PixelFormat;
using generic_cam::rotate_bgr;
using generic_cam::yuyv_to_bgr;

namespace
{

constexpr uint32_t kWidth = 64;
constexpr uint32_t kHeight = 48;
const CaptureFormat kMjpeg{PixelFormat::Mjpeg, kWidth, kHeight, 30};
const CaptureFormat kYuyv{PixelFormat::Yuyv, kWidth, kHeight, 30};

std::vector<uint8_t> gradient_bgr()
{
  std::vector<uint8_t> bgr(std::size_t{kWidth} * kHeight * 3);
  for (uint32_t y = 0; y < kHeight; ++y) {
    for (uint32_t x = 0; x < kWidth; ++x) {
      uint8_t * pixel = bgr.data() + (std::size_t{y} * kWidth + x) * 3;
      pixel[0] = static_cast<uint8_t>(x * 4);
      pixel[1] = static_cast<uint8_t>(y * 5);
      pixel[2] = 128;
    }
  }
  return bgr;
}

std::vector<uint8_t> encode_jpeg(std::vector<uint8_t> bgr)
{
  jpeg_compress_struct info{};
  jpeg_error_mgr error{};
  info.err = jpeg_std_error(&error);
  jpeg_create_compress(&info);
  unsigned char * buffer = nullptr;
  unsigned long size = 0;  // NOLINT(runtime/int): the jpeg_mem_dest signature
  jpeg_mem_dest(&info, &buffer, &size);
  info.image_width = kWidth;
  info.image_height = kHeight;
  info.input_components = 3;
  info.in_color_space = JCS_EXT_BGR;
  jpeg_set_defaults(&info);
  jpeg_set_quality(&info, 95, TRUE);
  jpeg_start_compress(&info, TRUE);
  while (info.next_scanline < kHeight) {
    JSAMPROW row = bgr.data() + std::size_t{info.next_scanline} * kWidth * 3;
    jpeg_write_scanlines(&info, &row, 1);
  }
  jpeg_finish_compress(&info);
  jpeg_destroy_compress(&info);
  std::vector<uint8_t> jpeg(buffer, buffer + size);
  std::free(buffer);
  return jpeg;
}

int max_difference(const std::vector<uint8_t> & a, const std::vector<uint8_t> & b)
{
  int worst = 0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    worst = std::max(worst, std::abs(a[i] - b[i]));
  }
  return worst;
}

class JpegDecoders : public ::testing::TestWithParam<std::string>
{
};

}  // namespace

TEST_P(JpegDecoders, RoundTripsAnImage)
{
  const auto original = gradient_bgr();
  const auto jpeg = encode_jpeg(original);
  auto decoder = make_decoder(kMjpeg, GetParam());
  std::vector<uint8_t> decoded(original.size());
  ASSERT_TRUE(decoder->decode(jpeg.data(), jpeg.size(), decoded.data()));
  EXPECT_LT(max_difference(original, decoded), 40);
  ASSERT_TRUE(decoder->decode(jpeg.data(), jpeg.size(), decoded.data()));
}

TEST_P(JpegDecoders, RefusesAnEmptyFrame)
{
  auto decoder = make_decoder(kMjpeg, GetParam());
  std::vector<uint8_t> decoded(std::size_t{kWidth} * kHeight * 3);
  const uint8_t none = 0;
  EXPECT_FALSE(decoder->decode(&none, 0, decoded.data()));
}

INSTANTIATE_TEST_SUITE_P(Backends, JpegDecoders, ::testing::Values("libjpeg", "jpegdec"));

TEST(LibjpegDecoder, RefusesGarbageAndKeepsWorking)
{
  const auto original = gradient_bgr();
  const auto jpeg = encode_jpeg(original);
  auto decoder = make_decoder(kMjpeg, "libjpeg");
  std::vector<uint8_t> decoded(original.size());
  const std::vector<uint8_t> garbage(500, 0x55);
  EXPECT_FALSE(decoder->decode(garbage.data(), garbage.size(), decoded.data()));
  EXPECT_TRUE(decoder->decode(jpeg.data(), jpeg.size(), decoded.data()));
}

TEST(LibjpegDecoder, RefusesATruncatedFrame)
{
  const auto original = gradient_bgr();
  const auto jpeg = encode_jpeg(original);
  auto decoder = make_decoder(kMjpeg, "libjpeg");
  std::vector<uint8_t> decoded(original.size());
  EXPECT_FALSE(decoder->decode(jpeg.data(), jpeg.size() * 2 / 3, decoded.data()));
  EXPECT_TRUE(decoder->decode(jpeg.data(), jpeg.size(), decoded.data()));
}

TEST(LibjpegDecoder, RefusesAnotherResolution)
{
  const auto jpeg = encode_jpeg(gradient_bgr());
  const CaptureFormat wider{PixelFormat::Mjpeg, kWidth * 2, kHeight, 30};
  auto decoder = make_decoder(wider, "libjpeg");
  std::vector<uint8_t> decoded(std::size_t{kWidth} * 2 * kHeight * 3);
  EXPECT_FALSE(decoder->decode(jpeg.data(), jpeg.size(), decoded.data()));
}

TEST(GstreamerDecoder, UnknownElementFailsAtConstruction)
{
  EXPECT_THROW(make_decoder(kMjpeg, "no_such_decoder_element"), std::runtime_error);
}

TEST(YuyvDecoder, RefusesAShortBuffer)
{
  auto decoder = make_decoder(kYuyv, "libjpeg");
  std::vector<uint8_t> yuyv(std::size_t{kWidth} * kHeight * 2 - 1);
  std::vector<uint8_t> decoded(std::size_t{kWidth} * kHeight * 3);
  EXPECT_FALSE(decoder->decode(yuyv.data(), yuyv.size(), decoded.data()));
}

TEST(YuyvToBgr, ConvertsReferenceColors)
{
  // Limited range BT.601: black, white and pure red.
  const std::vector<uint8_t> yuyv{16, 128, 235, 128, 81, 90, 81, 240};
  std::vector<uint8_t> bgr(12);
  yuyv_to_bgr(yuyv.data(), 2, 2, bgr.data());
  EXPECT_EQ(std::vector<uint8_t>(bgr.begin(), bgr.begin() + 3), (std::vector<uint8_t>{0, 0, 0}));
  EXPECT_EQ(
    std::vector<uint8_t>(bgr.begin() + 3, bgr.begin() + 6), (std::vector<uint8_t>{255, 255, 255}));
  EXPECT_NEAR(bgr[6], 0, 3);
  EXPECT_NEAR(bgr[7], 0, 3);
  EXPECT_NEAR(bgr[8], 255, 3);
}

TEST(RotateBgr, MovesCornersClockwise)
{
  // Two by three image with pixels labelled by their blue byte.
  const std::vector<uint8_t> src{1, 0, 0, 2, 0, 0, 3, 0, 0, 4, 0, 0, 5, 0, 0, 6, 0, 0};
  std::vector<uint8_t> dst(src.size());
  const auto blues = [&dst] {
    std::vector<uint8_t> out;
    for (std::size_t i = 0; i < dst.size(); i += 3) {
      out.push_back(dst[i]);
    }
    return out;
  };
  rotate_bgr(src.data(), 3, 2, 90, dst.data());
  EXPECT_EQ(blues(), (std::vector<uint8_t>{4, 1, 5, 2, 6, 3}));
  rotate_bgr(src.data(), 3, 2, 180, dst.data());
  EXPECT_EQ(blues(), (std::vector<uint8_t>{6, 5, 4, 3, 2, 1}));
  rotate_bgr(src.data(), 3, 2, 270, dst.data());
  EXPECT_EQ(blues(), (std::vector<uint8_t>{3, 6, 2, 5, 1, 4}));
}
