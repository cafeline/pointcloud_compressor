// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/core/BlockSizeReportFormatter.hpp"

using pointcloud_compressor::BlockSizeOptimizationResult;
using pointcloud_compressor::formatBlockSizeSummary;

TEST(BlockSizeReportFormatterTest, IncludesOptimalValues) {
    BlockSizeOptimizationResult result;
    result.optimal_block_size = 12;
    result.best_compression_ratio = 0.42f;
    result.tested_results.emplace(8, 0.5f);
    result.tested_results.emplace(12, 0.42f);

    const std::string summary = formatBlockSizeSummary(result, true);
    EXPECT_NE(summary.find("12"), std::string::npos);
    EXPECT_NE(summary.find("0.420000"), std::string::npos);
}

TEST(BlockSizeReportFormatterTest, HandlesFailureGracefully) {
    BlockSizeOptimizationResult result;
    result.optimal_block_size = -1;
    const std::string summary = formatBlockSizeSummary(result, false);
    EXPECT_NE(summary.find("failed"), std::string::npos);
}
