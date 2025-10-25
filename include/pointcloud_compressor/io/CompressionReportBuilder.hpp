// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_IO_COMPRESSION_REPORT_BUILDER_HPP
#define POINTCLOUD_COMPRESSOR_IO_COMPRESSION_REPORT_BUILDER_HPP

#include <string>
#include <vector>

#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/io/HDF5IO.hpp"
#include "pointcloud_compressor/bridge/RuntimeAPI.hpp"

namespace pointcloud_compressor::io {

class CompressionReportBuilder {
public:
    CompressionReportBuilder() = default;

    PCCCompressionReport build(const CompressionResult& result,
                               const PCCCompressionRequest& request,
                               std::vector<uint8_t>& dictionary_buffer,
                               std::vector<uint8_t>& indices_buffer,
                               std::vector<uint8_t>& occupancy_buffer,
                               std::string& transient_error_message) const;

    pointcloud_compressor::CompressedMapData toCompressedMapData(
        const CompressionResult& result,
        const PCCCompressionRequest& request,
        const std::vector<uint8_t>& dictionary_buffer,
        const std::vector<uint32_t>& block_indices_u32) const;

    pointcloud_compressor::CompressedMapData toCompressedMapData(const PCCCompressionReport& report,
                                                                 double voxel_size,
                                                                 int block_size) const;
    std::vector<uint8_t> extractOccupancy(const PCCCompressionReport& report) const;
};

}  // namespace pointcloud_compressor::io

#endif  // POINTCLOUD_COMPRESSOR_IO_COMPRESSION_REPORT_BUILDER_HPP
