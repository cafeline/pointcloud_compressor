// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_IO_HDF5_WRITERS_HPP
#define POINTCLOUD_COMPRESSOR_IO_HDF5_WRITERS_HPP

#include <string>
#include <vector>

#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/bridge/Bridge.hpp"

namespace pointcloud_compressor {
struct CompressedMapData;
}

namespace pointcloud_compressor::io {

bool writeCompressedMap(const std::string& output_path,
                        const pointcloud_compressor::CompressedMapData& data,
                        std::string& error_message);

bool writeRawVoxelGrid(const std::string& output_path,
                       const CompressionResult& result,
                       const PCCCompressionRequest& request,
                       const std::vector<uint8_t>& occupancy_buffer,
                       std::string& error_message);

bool writeRawVoxelGrid(const std::string& output_path,
                       const PCCCompressionReport& report,
                       std::string& error_message);

}  // namespace pointcloud_compressor::io

#endif  // POINTCLOUD_COMPRESSOR_IO_HDF5_WRITERS_HPP
