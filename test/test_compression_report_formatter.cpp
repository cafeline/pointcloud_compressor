// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "pointcloud_compressor/core/CompressionReportFormatter.hpp"

using pointcloud_compressor::formatCompressionSummary;

TEST(CompressionReportFormatterTest, IncludesKeyFields) {
    PCCCompressionReport report{};
    report.statistics.compression_ratio = 0.42;
    report.indices.total_blocks = 128;
    report.dictionary.num_patterns = 16;
    report.dictionary.pattern_size_bytes = 64;
    report.indices.index_bit_size = 8;
    auto summary = formatCompressionSummary(report);

    EXPECT_NE(summary.find("Compression summary"), std::string::npos);
    EXPECT_NE(summary.find("0.420000"), std::string::npos);
    EXPECT_NE(summary.find("128"), std::string::npos);
    EXPECT_NE(summary.find("16 patterns"), std::string::npos);
    EXPECT_NE(summary.find("8 bits"), std::string::npos);
}
