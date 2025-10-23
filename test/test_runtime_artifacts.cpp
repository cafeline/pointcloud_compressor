// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <vector>

#include "pointcloud_compressor/runtime/CompressionArtifacts.hpp"
#include "pointcloud_compressor/model/VoxelGrid.hpp"

using pointcloud_compressor::VoxelGrid;
using pointcloud_compressor::runtime::flattenDictionaryPatterns;
using pointcloud_compressor::runtime::packBlockIndices;
using pointcloud_compressor::runtime::buildOccupancyMask;

TEST(RuntimeArtifactsTest, FlattenDictionaryConcatenatesPatterns) {
    std::vector<std::vector<uint8_t>> patterns{
        {0x01, 0x02},
        {0xAA},
        {},
        {0xFF, 0xFE, 0xFD}
    };

    const auto flattened = flattenDictionaryPatterns(patterns);
    const std::vector<uint8_t> expected{0x01, 0x02, 0xAA, 0xFF, 0xFE, 0xFD};
    EXPECT_EQ(expected, flattened);
}

TEST(RuntimeArtifactsTest, PackBlockIndicesHandlesVariousWidths) {
    const std::vector<uint64_t> indices{0x01u, 0xFFu, 0xABCDEu};

    const auto packed8 = packBlockIndices(indices, 8);
    ASSERT_EQ(3u, packed8.size());
    EXPECT_EQ(0x01u, packed8[0]);
    EXPECT_EQ(0xFFu, packed8[1]);
    EXPECT_EQ(0xDEu, packed8[2]);

    const auto packed16 = packBlockIndices(indices, 16);
    ASSERT_EQ(6u, packed16.size());
    EXPECT_EQ(0x01u, packed16[0]);
    EXPECT_EQ(0x00u, packed16[1]);
    EXPECT_EQ(0xFFu, packed16[2]);
    EXPECT_EQ(0x00u, packed16[3]);
    EXPECT_EQ(0xDEu, packed16[4]);
    EXPECT_EQ(0xBCu, packed16[5]);
}

TEST(RuntimeArtifactsTest, BuildOccupancyMaskReflectsGridState) {
    VoxelGrid grid;
    grid.initialize(2, 2, 2, 0.1f);
    grid.setVoxel(0, 0, 0, true);
    grid.setVoxel(1, 0, 0, false);
    grid.setVoxel(0, 1, 1, true);

    const auto mask = buildOccupancyMask(grid);
    ASSERT_EQ(8u, mask.size());
    EXPECT_EQ(1u, mask[0]);
    EXPECT_EQ(0u, mask[1]);
    EXPECT_EQ(0u, mask[2]);
    EXPECT_EQ(0u, mask[3]);
    EXPECT_EQ(0u, mask[4]);
    EXPECT_EQ(0u, mask[5]);
    EXPECT_EQ(1u, mask[6]);
    EXPECT_EQ(0u, mask[7]);
}
