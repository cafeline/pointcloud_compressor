// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_RUNTIME_RUNTIME_HELPERS_HPP
#define POINTCLOUD_COMPRESSOR_RUNTIME_RUNTIME_HELPERS_HPP

#include <cstdint>
#include <vector>

#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/io/HDF5IO.hpp"
#include "pointcloud_compressor/runtime/RuntimeAPI.hpp"

namespace pointcloud_compressor::runtime {

uint8_t bitWidthFromMaxIndex(uint64_t max_index);
std::vector<uint32_t> convertBlockIndicesToU32(const std::vector<uint64_t>& indices);
void populateCompressedMapData(const CompressionResult& result,
                               const PCCCompressionRequest& request,
                               const std::vector<uint8_t>& dictionary_buffer,
                               const std::vector<uint32_t>& block_indices_u32,
                               pointcloud_compressor::CompressedMapData& data);

}  // namespace pointcloud_compressor::runtime

#endif  // POINTCLOUD_COMPRESSOR_RUNTIME_RUNTIME_HELPERS_HPP
