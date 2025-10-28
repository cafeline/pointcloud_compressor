#include <gtest/gtest.h>

#include "vq_occupancy_compressor/common/CompressionDataUtils.hpp"

TEST(CompressionDataUtils, BitWidthFromMaxIndex) {
  using vq_occupancy_compressor::common::bitWidthFromMaxIndex;

  EXPECT_EQ(bitWidthFromMaxIndex(0), 1);
  EXPECT_EQ(bitWidthFromMaxIndex(1), 1);
  EXPECT_EQ(bitWidthFromMaxIndex(2), 2);
  EXPECT_EQ(bitWidthFromMaxIndex(3), 2);
  EXPECT_EQ(bitWidthFromMaxIndex(255), 8);
  EXPECT_EQ(bitWidthFromMaxIndex(256), 9);
  EXPECT_EQ(bitWidthFromMaxIndex((1ULL << 32) - 1), 32);
}

TEST(CompressionDataUtils, ConvertBlockIndicesToU32) {
  using vq_occupancy_compressor::common::convertBlockIndicesToU32;
  std::vector<uint64_t> src{0, 1, 65535, 1ULL << 33};
  auto converted = convertBlockIndicesToU32(src);
  ASSERT_EQ(converted.size(), src.size());
  EXPECT_EQ(converted[0], 0u);
  EXPECT_EQ(converted[1], 1u);
  EXPECT_EQ(converted[2], 65535u);
  EXPECT_EQ(converted[3], static_cast<uint32_t>(1ULL << 33));
}
