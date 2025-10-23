// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_RUNTIME_COMPRESSION_ARTIFACTS_HPP
#define POINTCLOUD_COMPRESSOR_RUNTIME_COMPRESSION_ARTIFACTS_HPP

#include <cstdint>
#include <vector>

namespace pointcloud_compressor {
class VoxelGrid;
}

namespace pointcloud_compressor::runtime {

std::vector<uint8_t> flattenDictionaryPatterns(const std::vector<std::vector<uint8_t>>& patterns);
std::vector<uint8_t> packBlockIndices(const std::vector<uint64_t>& indices, uint8_t bit_size);
std::vector<uint8_t> buildOccupancyMask(const pointcloud_compressor::VoxelGrid& grid);

}  // namespace pointcloud_compressor::runtime

#endif  // POINTCLOUD_COMPRESSOR_RUNTIME_COMPRESSION_ARTIFACTS_HPP
