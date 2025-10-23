// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_compressor/runtime/CompressionArtifacts.hpp"

#include "pointcloud_compressor/model/VoxelGrid.hpp"

#include <algorithm>
#include <vector>

namespace pointcloud_compressor::runtime {

std::vector<uint8_t> flattenDictionaryPatterns(const std::vector<std::vector<uint8_t>>& patterns) {
    if (patterns.empty()) {
        return {};
    }

    std::size_t total_bytes = 0;
    for (const auto& pattern : patterns) {
        total_bytes += pattern.size();
    }

    std::vector<uint8_t> buffer;
    buffer.reserve(total_bytes);
    for (const auto& pattern : patterns) {
        buffer.insert(buffer.end(), pattern.begin(), pattern.end());
    }
    return buffer;
}

std::vector<uint8_t> packBlockIndices(const std::vector<uint64_t>& indices, uint8_t bit_size) {
    std::vector<uint8_t> packed;
    if (indices.empty()) {
        return packed;
    }

    switch (bit_size) {
        case 8: {
            packed.reserve(indices.size());
            for (uint64_t value : indices) {
                packed.push_back(static_cast<uint8_t>(value & 0xFFu));
            }
            break;
        }
        case 16: {
            packed.reserve(indices.size() * 2);
            for (uint64_t value : indices) {
                packed.push_back(static_cast<uint8_t>(value & 0xFFu));
                packed.push_back(static_cast<uint8_t>((value >> 8) & 0xFFu));
            }
            break;
        }
        case 32: {
            packed.reserve(indices.size() * 4);
            for (uint64_t value : indices) {
                packed.push_back(static_cast<uint8_t>(value & 0xFFu));
                packed.push_back(static_cast<uint8_t>((value >> 8) & 0xFFu));
                packed.push_back(static_cast<uint8_t>((value >> 16) & 0xFFu));
                packed.push_back(static_cast<uint8_t>((value >> 24) & 0xFFu));
            }
            break;
        }
        case 64: {
            packed.reserve(indices.size() * 8);
            for (uint64_t value : indices) {
                packed.push_back(static_cast<uint8_t>(value & 0xFFu));
                packed.push_back(static_cast<uint8_t>((value >> 8) & 0xFFu));
                packed.push_back(static_cast<uint8_t>((value >> 16) & 0xFFu));
                packed.push_back(static_cast<uint8_t>((value >> 24) & 0xFFu));
                packed.push_back(static_cast<uint8_t>((value >> 32) & 0xFFu));
                packed.push_back(static_cast<uint8_t>((value >> 40) & 0xFFu));
                packed.push_back(static_cast<uint8_t>((value >> 48) & 0xFFu));
                packed.push_back(static_cast<uint8_t>((value >> 56) & 0xFFu));
            }
            break;
        }
        default:
            // Unsupported bit size; return empty vector
            break;
    }

    return packed;
}

std::vector<uint8_t> buildOccupancyMask(const pointcloud_compressor::VoxelGrid& grid) {
    const auto dims = grid.getDimensions();
    if (dims.x <= 0 || dims.y <= 0 || dims.z <= 0) {
        return {};
    }

    const std::size_t total_voxels =
        static_cast<std::size_t>(dims.x) *
        static_cast<std::size_t>(dims.y) *
        static_cast<std::size_t>(dims.z);

    std::vector<uint8_t> occupancy;
    occupancy.reserve(total_voxels);

    for (int z = 0; z < dims.z; ++z) {
        for (int y = 0; y < dims.y; ++y) {
            for (int x = 0; x < dims.x; ++x) {
                occupancy.push_back(grid.getVoxel(x, y, z) ? 1u : 0u);
            }
        }
    }

    return occupancy;
}

}  // namespace pointcloud_compressor::runtime
