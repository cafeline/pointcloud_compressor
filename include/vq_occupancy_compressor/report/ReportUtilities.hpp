// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_REPORT_REPORT_UTILITIES_HPP
#define VQ_OCCUPANCY_COMPRESSOR_REPORT_REPORT_UTILITIES_HPP

#include <string>
#include <vector>

#include "vq_occupancy_compressor/bridge/Bridge.hpp"
#include "vq_occupancy_compressor/core/VqOccupancyCompressor.hpp"
#include "vq_occupancy_compressor/io/HDF5IO.hpp"

namespace vq_occupancy_compressor {

std::string formatCompressionSummary(const PCCCompressionReport& report);

std::string formatBlockSizeSummary(const BlockSizeOptimizationResult& result,
                                   bool verbose);

}  // namespace vq_occupancy_compressor

namespace vq_occupancy_compressor::io {

class CompressionReportBuilder {
public:
    CompressionReportBuilder() = default;

    PCCCompressionReport build(const CompressionResult& result,
                               const PCCCompressionRequest& request,
                               std::vector<uint8_t>& dictionary_buffer,
                               std::vector<uint8_t>& indices_buffer,
                               std::vector<uint8_t>& occupancy_buffer,
                               std::string& transient_error_message) const;

    vq_occupancy_compressor::CompressedMapData toCompressedMapData(
        const CompressionResult& result,
        const PCCCompressionRequest& request,
        const std::vector<uint8_t>& dictionary_buffer,
        const std::vector<uint32_t>& block_indices_u32) const;

    vq_occupancy_compressor::CompressedMapData toCompressedMapData(
        const PCCCompressionReport& report,
        double voxel_size,
        int block_size) const;

    std::vector<uint8_t> extractOccupancy(const PCCCompressionReport& report) const;
};

}  // namespace vq_occupancy_compressor::io

#endif  // VQ_OCCUPANCY_COMPRESSOR_REPORT_REPORT_UTILITIES_HPP
