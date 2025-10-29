// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "vq_occupancy_compressor/ros/BlockSizeOptimizerSummary.hpp"
#include "vq_occupancy_compressor/model/VoxelGrid.hpp"
#include "vq_occupancy_compressor/utils/CompressionSummary.hpp"
#include <cctype>

using vq_occupancy_compressor::CompressionResult;
using vq_occupancy_compressor::ros::buildOptimizationSummaryJson;
using vq_occupancy_compressor::utils::computeSummaryMetrics;

TEST(BlockSizeOptimizerSummary, UsesActualCompressionResult) {
    CompressionResult result;
    result.success = true;
    result.voxel_grid.initialize(2, 2, 2, 0.1f);
    result.block_size = 2;
    result.blocks_count = {1, 1, 1};
    result.index_bit_size = 8;
    result.block_indices = {0, 1, 2, 3};
    result.pattern_dictionary = {std::vector<uint8_t>(4, 0x01)};
    result.compressed_size = 8;

    const auto metrics = computeSummaryMetrics(result);
    const std::string json = buildOptimizationSummaryJson(8, metrics);

    auto extractInt = [](const std::string& src, const std::string& key) {
        const auto pos = src.find(key);
        EXPECT_NE(pos, std::string::npos);
        auto value_pos = src.find(":", pos);
        EXPECT_NE(value_pos, std::string::npos);
        value_pos += 1;
        while (value_pos < src.size() && std::isspace(static_cast<unsigned char>(src[value_pos]))) {
            ++value_pos;
        }
        auto end = value_pos;
        while (end < src.size() && std::isdigit(static_cast<unsigned char>(src[end]))) {
            ++end;
        }
        return std::stoi(src.substr(value_pos, end - value_pos));
    };

    EXPECT_EQ(extractInt(json, "\"optimal_block_size\""), 8);
    EXPECT_EQ(extractInt(json, "\"dictionary_entries\""), 1);
    EXPECT_EQ(extractInt(json, "\"index_entries\""), 4);
    EXPECT_NE(json.find("\"compression_ratio_vs_one_byte\""), std::string::npos);
    EXPECT_NE(json.find("\"compression_ratio_vs_one_bit\""), std::string::npos);
    EXPECT_DOUBLE_EQ(metrics.ratio_vs_one_byte, 1.0);
    EXPECT_DOUBLE_EQ(metrics.ratio_vs_one_bit, 8.0);
}
