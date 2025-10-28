// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_COMMON_COMPRESSION_ARTIFACTS_HPP
#define VQ_OCCUPANCY_COMPRESSOR_COMMON_COMPRESSION_ARTIFACTS_HPP

#include <cstdint>
#include <vector>

namespace vq_occupancy_compressor {
class VoxelGrid;
}

namespace vq_occupancy_compressor::common {

std::vector<uint8_t> flattenDictionaryPatterns(const std::vector<std::vector<uint8_t>>& patterns);
std::vector<uint8_t> packBlockIndices(const std::vector<uint64_t>& indices, uint8_t bit_size);
std::vector<uint8_t> buildOccupancyMask(const vq_occupancy_compressor::VoxelGrid& grid);

}  // namespace vq_occupancy_compressor::common

#endif  // VQ_OCCUPANCY_COMPRESSOR_COMMON_COMPRESSION_ARTIFACTS_HPP
