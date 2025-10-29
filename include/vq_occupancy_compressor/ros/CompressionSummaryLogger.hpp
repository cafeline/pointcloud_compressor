// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_ROS_COMPRESSION_SUMMARY_LOGGER_HPP
#define VQ_OCCUPANCY_COMPRESSOR_ROS_COMPRESSION_SUMMARY_LOGGER_HPP

#include <rclcpp/logger.hpp>

#include "vq_occupancy_compressor/utils/CompressionSummary.hpp"

namespace vq_occupancy_compressor::ros {

inline void logCompressionSummary(const rclcpp::Logger& logger,
                                  const utils::CompressionSummaryMetrics& metrics) {
    const auto bytesToMb = [](std::size_t bytes) {
        return static_cast<double>(bytes) / 1'000'000.0;
    };

    RCLCPP_INFO(logger,
                "Environment scale: %.2fm * %.2fm * %.2fm",
                metrics.env_x,
                metrics.env_y,
                metrics.env_z);
    RCLCPP_INFO(logger,
                "Map size at 1 byte/voxel (pre-compression): %.2f MB",
                bytesToMb(metrics.bytes_one_byte));
    RCLCPP_INFO(logger,
                "Map size at 1 bit/voxel (pre-compression): %.2f MB",
                bytesToMb(metrics.bytes_one_bit));
    RCLCPP_INFO(logger,
                "Index array: %zu entries, %.2f MB",
                metrics.index_entries,
                bytesToMb(metrics.index_bytes));
    RCLCPP_INFO(logger,
                "Codebook: %zu entries, %.2f MB",
                metrics.dictionary_entries,
                bytesToMb(metrics.dictionary_bytes));
    RCLCPP_INFO(logger,
                "Compressed map total: %.2f MB",
                bytesToMb(metrics.compressed_bytes));
    RCLCPP_INFO(logger,
                "Compression ratio vs 1 byte/voxel: %.6f",
                metrics.ratio_vs_one_byte);
    RCLCPP_INFO(logger,
                "Compression ratio vs 1 bit/voxel: %.6f",
                metrics.ratio_vs_one_bit);
}

}  // namespace vq_occupancy_compressor::ros

#endif
