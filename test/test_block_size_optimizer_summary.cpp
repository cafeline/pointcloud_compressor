// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "vq_occupancy_compressor/ros/BlockSizeOptimizerSummary.hpp"
#include "vq_occupancy_compressor/model/VoxelGrid.hpp"
#include "vq_occupancy_compressor/utils/CompressionSummary.hpp"

using vq_occupancy_compressor::CompressionResult;
using vq_occupancy_compressor::ros::buildOptimizationSummaryJson;
using vq_occupancy_compressor::utils::computeSummaryMetrics;

TEST(BlockSizeOptimizerSummary, UsesActualCompressionResult) {
    CompressionResult result;
    result.success = true;
    result.voxel_grid.initialize(2, 2, 2, 0.1f);
    result.block_size = 2;
    result.index_bit_size = 8;
    result.block_indices = {0, 1, 2, 3};
    result.pattern_dictionary = {std::vector<uint8_t>(4, 0x01)};
    result.compressed_size = 8;

    const auto metrics = computeSummaryMetrics(result);
    const std::string json = buildOptimizationSummaryJson(8, metrics);

    EXPECT_NE(json.find("\"optimal_block_size\": 8"), std::string::npos);
    EXPECT_NE(json.find("\"dictionary_entries\": 1"), std::string::npos);
    EXPECT_NE(json.find("\"index_entries\": 4"), std::string::npos);
    EXPECT_NE(json.find("\"compression_ratio_vs_one_byte\": 1.000000"), std::string::npos);
    EXPECT_NE(json.find("\"compression_ratio_vs_one_bit\": 8.000000"), std::string::npos);
}
