// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_IO_HDF5_WRITERS_HPP
#define VQ_OCCUPANCY_COMPRESSOR_IO_HDF5_WRITERS_HPP

#include <string>
#include <vector>

#include "vq_occupancy_compressor/core/VqOccupancyCompressor.hpp"
#include "vq_occupancy_compressor/bridge/Bridge.hpp"

namespace vq_occupancy_compressor {
struct CompressedMapData;
}

namespace vq_occupancy_compressor::io {

bool writeCompressedMap(const std::string& output_path,
                        const vq_occupancy_compressor::CompressedMapData& data,
                        std::string& error_message);

bool writeRawVoxelGrid(const std::string& output_path,
                       const CompressionResult& result,
                       const PCCCompressionRequest& request,
                       const std::vector<uint8_t>& occupancy_buffer,
                       std::string& error_message);

bool writeRawVoxelGrid(const std::string& output_path,
                       const PCCCompressionReport& report,
                       std::string& error_message);

}  

#endif  
