// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_compressor/runtime/RuntimeHelpers.hpp"

#include <algorithm>

namespace pointcloud_compressor::runtime {

namespace {
constexpr uint8_t kMaxStoredBitWidth = 32;
}  // namespace

uint8_t bitWidthFromMaxIndex(uint64_t max_index) {
    if (max_index == 0) {
        return 1;
    }

    uint8_t bits = 0;
    while (max_index > 0) {
        ++bits;
        max_index >>= 1;
    }

    return std::min<uint8_t>(bits, kMaxStoredBitWidth);
}

std::vector<uint32_t> convertBlockIndicesToU32(const std::vector<uint64_t>& indices) {
    std::vector<uint32_t> converted;
    converted.reserve(indices.size());
    for (uint64_t value : indices) {
        converted.push_back(static_cast<uint32_t>(value));
    }
    return converted;
}

void populateCompressedMapData(const CompressionResult& result,
                               const PCCCompressionRequest& request,
                               const std::vector<uint8_t>& dictionary_buffer,
                               const std::vector<uint32_t>& block_indices_u32,
                               pointcloud_compressor::CompressedMapData& data) {
    data.voxel_size = static_cast<float>(request.voxel_size);
    data.dictionary_size = static_cast<uint32_t>(result.num_unique_patterns);
    data.block_size = static_cast<uint32_t>(result.block_size);

    const uint32_t pattern_bits = static_cast<uint32_t>(result.block_size) *
                                  static_cast<uint32_t>(result.block_size) *
                                  static_cast<uint32_t>(result.block_size);
    data.pattern_bits = pattern_bits;
    data.pattern_length = pattern_bits;

    data.dictionary_patterns.assign(dictionary_buffer.begin(), dictionary_buffer.end());

    float origin_x = 0.0f;
    float origin_y = 0.0f;
    float origin_z = 0.0f;
    result.voxel_grid.getOrigin(origin_x, origin_y, origin_z);
    data.grid_origin = {origin_x, origin_y, origin_z};

    const auto dims = result.voxel_grid.getDimensions();
    data.grid_dimensions = {
        static_cast<int32_t>(dims.x),
        static_cast<int32_t>(dims.y),
        static_cast<int32_t>(dims.z)};

    data.block_dims = {
        static_cast<int32_t>(result.blocks_count.x),
        static_cast<int32_t>(result.blocks_count.y),
        static_cast<int32_t>(result.blocks_count.z)};

    data.block_indices = block_indices_u32;

    data.block_index_bit_width = bitWidthFromMaxIndex(result.max_index);
    data.block_index_sentinel = 0;

    data.original_points = static_cast<uint64_t>(result.point_count);
    data.compressed_voxels = static_cast<uint64_t>(result.num_blocks) *
                             static_cast<uint64_t>(result.block_size) *
                             static_cast<uint64_t>(result.block_size) *
                             static_cast<uint64_t>(result.block_size);
    data.compression_ratio = result.compression_ratio;

    data.bounding_box_min = {
        result.grid_origin.x,
        result.grid_origin.y,
        result.grid_origin.z};

    data.bounding_box_max = {
        result.grid_origin.x + result.grid_dimensions.x,
        result.grid_origin.y + result.grid_dimensions.y,
        result.grid_origin.z + result.grid_dimensions.z};
}

}  // namespace pointcloud_compressor::runtime
