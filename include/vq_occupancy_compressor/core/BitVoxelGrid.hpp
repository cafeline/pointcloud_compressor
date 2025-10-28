// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_BIT_VOXEL_GRID_HPP
#define VQ_OCCUPANCY_COMPRESSOR_BIT_VOXEL_GRID_HPP

#include <vector>
#include <cstdint>
#include <algorithm>

namespace vq_occupancy_compressor {

class BitVoxelGrid {
private:
    std::vector<uint64_t> bit_vector;
    uint64_t total_voxels;
    int grid_x, grid_y, grid_z;

public:
    BitVoxelGrid(int x, int y, int z)
        : grid_x(x), grid_y(y), grid_z(z) {
        total_voxels = static_cast<uint64_t>(x) * y * z;
        size_t num_words = (total_voxels + 63) / 64;
        bit_vector.resize(num_words, 0);
    }

    inline void setBit(uint64_t idx) {
        if (idx >= total_voxels) return;
        bit_vector[idx >> 6] |= (1ULL << (idx & 63));
    }

    inline bool getBit(uint64_t idx) const {
        if (idx >= total_voxels) return false;
        return (bit_vector[idx >> 6] & (1ULL << (idx & 63))) != 0;
    }

    inline void clearBit(uint64_t idx) {
        if (idx >= total_voxels) return;
        bit_vector[idx >> 6] &= ~(1ULL << (idx & 63));
    }

    inline void setBit3D(int x, int y, int z) {
        uint64_t idx = voxelIndex3Dto1D(x, y, z);
        if (idx < total_voxels) setBit(idx);
    }

    inline bool getBit3D(int x, int y, int z) const {
        uint64_t idx = voxelIndex3Dto1D(x, y, z);
        return (idx < total_voxels) ? getBit(idx) : false;
    }

    inline uint64_t voxelIndex3Dto1D(int x, int y, int z) const {
        if (x < 0 || x >= grid_x || y < 0 || y >= grid_y || z < 0 || z >= grid_z) {
            return total_voxels; 
        }
        return static_cast<uint64_t>(z) * grid_x * grid_y +
               static_cast<uint64_t>(y) * grid_x +
               static_cast<uint64_t>(x);
    }

    size_t getMemoryUsageBytes() const {
        return bit_vector.size() * sizeof(uint64_t);
    }

    uint64_t getTotalVoxels() const {
        return total_voxels;
    }

    size_t countOccupiedVoxels() const {
        size_t count = 0;
        for (uint64_t word : bit_vector) {
            count += __builtin_popcountll(word);
        }
        return count;
    }

    void clear() {
        std::fill(bit_vector.begin(), bit_vector.end(), 0);
    }

    void getGridDimensions(int& x, int& y, int& z) const {
        x = grid_x;
        y = grid_y;
        z = grid_z;
    }

    inline bool isValidCoordinate(int x, int y, int z) const {
        return x >= 0 && x < grid_x &&
               y >= 0 && y < grid_y &&
               z >= 0 && z < grid_z;
    }
};

} 

#endif 