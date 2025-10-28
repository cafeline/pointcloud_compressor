// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_UTILS_COMPRESSION_SUMMARY_HPP
#define VQ_OCCUPANCY_COMPRESSOR_UTILS_COMPRESSION_SUMMARY_HPP

#include <cstddef>

#include "vq_occupancy_compressor/core/VqOccupancyCompressor.hpp"
#include "vq_occupancy_compressor/bridge/Bridge.hpp"

namespace vq_occupancy_compressor::utils {

struct CompressionSummaryMetrics {
    double env_x = 0.0;
    double env_y = 0.0;
    double env_z = 0.0;

    std::size_t total_voxels = 0;
    std::size_t bytes_one_byte = 0;
    std::size_t bytes_one_bit = 0;

    std::size_t dictionary_entries = 0;
    std::size_t dictionary_bytes = 0;

    std::size_t index_entries = 0;
    std::size_t index_bytes = 0;

    std::size_t compressed_bytes = 0;

    double ratio_vs_one_byte = 0.0;
    double ratio_vs_one_bit = 0.0;
};

CompressionSummaryMetrics computeSummaryMetrics(const CompressionResult& result);
CompressionSummaryMetrics computeSummaryMetrics(const PCCCompressionReport& report);

}  // namespace vq_occupancy_compressor::utils

#endif
