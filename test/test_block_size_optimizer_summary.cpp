// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "vq_occupancy_compressor/ros/BlockSizeOptimizerSummary.hpp"

using vq_occupancy_compressor::CompressionResult;
using vq_occupancy_compressor::ros::buildOptimizationSummaryJson;

TEST(BlockSizeOptimizerSummary, UsesActualCompressionResult) {
    CompressionResult result;
    result.success = true;
    result.compression_ratio = 0.25f;
    result.original_size = 400;
    result.compressed_size = 100;

    const std::string json = buildOptimizationSummaryJson(8, result);

    EXPECT_NE(json.find("\"optimal_block_size\": 8"), std::string::npos);
    EXPECT_NE(json.find("\"best_compression_ratio\": 0.250000"), std::string::npos);
    EXPECT_NE(json.find("\"compressed_size_bytes\": 100"), std::string::npos);
    EXPECT_NE(json.find("\"original_size_bytes\": 400"), std::string::npos);
    EXPECT_EQ(json.find("tested_results"), std::string::npos);
}
