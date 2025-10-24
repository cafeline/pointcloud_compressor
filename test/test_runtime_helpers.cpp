// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "pointcloud_compressor/config/CompressorConfig.hpp"
#include "pointcloud_compressor/runtime/RuntimeHelpers.hpp"
#include "pointcloud_compressor/runtime/CompressionReportBuilder.hpp"

using pointcloud_compressor::CompressionResult;
using pointcloud_compressor::CompressedMapData;
using pointcloud_compressor::VoxelGrid;
using pointcloud_compressor::runtime::CompressionReportBuilder;
using pointcloud_compressor::runtime::bitWidthFromMaxIndex;
using pointcloud_compressor::runtime::convertBlockIndicesToU32;

namespace {

PCCCompressionRequest makeRequest() {
    pointcloud_compressor::CompressionSettings settings;
    settings.voxel_size = 0.05f;
    settings.block_size = 2;
    settings.min_points_threshold = 1;

    pointcloud_compressor::config::CompressorConfig config;
    config.voxel_size = 0.05;
    config.block_size = 2;
    config.min_points_threshold = 1;
    config.save_hdf5 = false;
    config.save_raw_hdf5 = false;

    return pointcloud_compressor::config::toCompressionRequest(config, settings);
}

}  // namespace

TEST(RuntimeHelpersTest, BitWidthFromMaxIndexCapsAt32Bits) {
    EXPECT_EQ(1u, bitWidthFromMaxIndex(0));
    EXPECT_EQ(4u, bitWidthFromMaxIndex(15));
    EXPECT_EQ(32u, bitWidthFromMaxIndex(1ULL << 48));
}

TEST(RuntimeHelpersTest, PopulateCompressedMapDataUsesMaxIndex) {
    CompressionResult result;
    result.block_size = 2;
    result.num_unique_patterns = 1;
    result.pattern_dictionary = {{0xAA, 0xBB}};
    result.block_indices = {0, 0};
    result.max_index = 1024;  // Deliberately larger than any entry in block_indices
    result.point_count = 4;
    result.num_blocks = 2;
    result.block_size = 2;
    result.grid_dimensions = {2.0, 2.0, 2.0};
    result.grid_origin = {1.0, 2.0, 3.0};
    result.blocks_count = {1, 1, 2};
    result.compression_ratio = 0.5;
    result.voxel_grid.initialize(2, 2, 2, 0.05f);
    result.voxel_grid.setOrigin(1.0f, 2.0f, 3.0f);

    PCCCompressionRequest request = makeRequest();

    std::vector<uint8_t> dictionary_buffer = {0xAA, 0xBB};
    std::vector<uint32_t> block_indices_u32 = convertBlockIndicesToU32(result.block_indices);

    CompressionReportBuilder builder;
    auto data = builder.toCompressedMapData(result, request, dictionary_buffer, block_indices_u32);

    EXPECT_EQ(bitWidthFromMaxIndex(result.max_index), data.block_index_bit_width);
    EXPECT_EQ(dictionary_buffer.size(), data.dictionary_patterns.size());
    EXPECT_DOUBLE_EQ(result.grid_origin.x, data.bounding_box_min[0]);
    EXPECT_DOUBLE_EQ(result.grid_origin.x + result.grid_dimensions.x, data.bounding_box_max[0]);
}
