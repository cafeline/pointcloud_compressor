// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_ROS_BLOCK_SIZE_OPTIMIZER_SUMMARY_HPP
#define VQ_OCCUPANCY_COMPRESSOR_ROS_BLOCK_SIZE_OPTIMIZER_SUMMARY_HPP

#include <string>
#include <iomanip>
#include <sstream>

#include "vq_occupancy_compressor/utils/CompressionSummary.hpp"

namespace vq_occupancy_compressor::ros {

inline std::string buildOptimizationSummaryJson(int optimal_block_size,
                                                const utils::CompressionSummaryMetrics& metrics) {
    const auto bytesToMb = [](std::size_t bytes) {
        return static_cast<double>(bytes) / (1024.0 * 1024.0);
    };

    std::ostringstream ss;
    ss << "{\n";
    ss << "  \"optimal_block_size\": " << optimal_block_size << ",\n";
    ss << "  \"dictionary_entries\": " << metrics.dictionary_entries << ",\n";
    ss << "  \"dictionary_size_mb\": " << std::fixed << std::setprecision(6)
       << bytesToMb(metrics.dictionary_bytes) << ",\n";
    ss << "  \"index_entries\": " << metrics.index_entries << ",\n";
    ss << "  \"index_size_mb\": " << std::fixed << std::setprecision(6)
       << bytesToMb(metrics.index_bytes) << ",\n";
    ss << "  \"compressed_size_mb\": " << std::fixed << std::setprecision(6)
       << bytesToMb(metrics.compressed_bytes) << ",\n";
    ss << "  \"map_size_one_byte_mb\": " << std::fixed << std::setprecision(6)
       << bytesToMb(metrics.bytes_one_byte) << ",\n";
    ss << "  \"map_size_one_bit_mb\": " << std::fixed << std::setprecision(6)
       << bytesToMb(metrics.bytes_one_bit) << ",\n";
    ss << "  \"compression_ratio_vs_one_byte\": " << std::fixed << std::setprecision(6)
       << metrics.ratio_vs_one_byte << ",\n";
    ss << "  \"compression_ratio_vs_one_bit\": " << std::fixed << std::setprecision(6)
       << metrics.ratio_vs_one_bit << "\n";
    ss << "}";
    return ss.str();
}

}  // namespace vq_occupancy_compressor::ros

#endif
