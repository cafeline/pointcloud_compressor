// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_ROS_BLOCK_SIZE_OPTIMIZER_SUMMARY_HPP
#define VQ_OCCUPANCY_COMPRESSOR_ROS_BLOCK_SIZE_OPTIMIZER_SUMMARY_HPP

#include <string>

#include "vq_occupancy_compressor/core/VqOccupancyCompressor.hpp"

#include <iomanip>
#include <sstream>

namespace vq_occupancy_compressor::ros {

inline std::string buildOptimizationSummaryJson(int optimal_block_size,
                                                const CompressionResult& compression_result) {
    std::ostringstream ss;
    ss << "{\n";
    ss << "  \"optimal_block_size\": " << optimal_block_size << ",\n";
    ss << "  \"best_compression_ratio\": " << std::fixed << std::setprecision(6)
       << compression_result.compression_ratio << ",\n";
    ss << "  \"compressed_size_bytes\": " << compression_result.compressed_size << ",\n";
    ss << "  \"original_size_bytes\": " << compression_result.original_size << "\n";
    ss << "}";
    return ss.str();
}

}  // namespace vq_occupancy_compressor::ros

#endif
