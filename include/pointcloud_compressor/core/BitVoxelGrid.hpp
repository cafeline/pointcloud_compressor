// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_BIT_VOXEL_GRID_HPP
#define POINTCLOUD_COMPRESSOR_BIT_VOXEL_GRID_HPP

#include <vector>
#include <cstdint>
#include <algorithm>

namespace pointcloud_compressor {

/**
 * @brief Memory-efficient voxel grid using bit vectors
 * 
 * This class provides a compact representation of a 3D voxel grid
 * using only 1 bit per voxel, achieving ~97% memory reduction
 * compared to traditional int arrays.
 */
class BitVoxelGrid {
private:
    std::vector<uint64_t> bit_vector;  ///< Bit storage (64 voxels per uint64_t)
    uint64_t total_voxels;              ///< Total number of voxels
    int grid_x, grid_y, grid_z;        ///< Grid dimensions
    
public:
    /**
     * @brief Construct a new Bit Voxel Grid
     * @param x Grid size in X dimension
     * @param y Grid size in Y dimension
     * @param z Grid size in Z dimension
     */
    BitVoxelGrid(int x, int y, int z) 
        : grid_x(x), grid_y(y), grid_z(z) {
        total_voxels = static_cast<uint64_t>(x) * y * z;
        size_t num_words = (total_voxels + 63) / 64;
        bit_vector.resize(num_words, 0);
    }
    
    /**
     * @brief Set a bit at the given index
     * @param idx Linear voxel index
     */
    inline void setBit(uint64_t idx) {
        if (idx >= total_voxels) return;
        bit_vector[idx >> 6] |= (1ULL << (idx & 63));
    }
    
    /**
     * @brief Get bit value at the given index
     * @param idx Linear voxel index
     * @return true if bit is set, false otherwise
     */
    inline bool getBit(uint64_t idx) const {
        if (idx >= total_voxels) return false;
        return (bit_vector[idx >> 6] & (1ULL << (idx & 63))) != 0;
    }
    
    /**
     * @brief Clear a bit at the given index
     * @param idx Linear voxel index
     */
    inline void clearBit(uint64_t idx) {
        if (idx >= total_voxels) return;
        bit_vector[idx >> 6] &= ~(1ULL << (idx & 63));
    }
    
    /**
     * @brief Set bit at 3D coordinates
     * @param x X coordinate
     * @param y Y coordinate
     * @param z Z coordinate
     */
    inline void setBit3D(int x, int y, int z) {
        uint64_t idx = voxelIndex3Dto1D(x, y, z);
        if (idx < total_voxels) setBit(idx);
    }
    
    /**
     * @brief Get bit at 3D coordinates
     * @param x X coordinate
     * @param y Y coordinate
     * @param z Z coordinate
     * @return true if voxel is occupied, false otherwise
     */
    inline bool getBit3D(int x, int y, int z) const {
        uint64_t idx = voxelIndex3Dto1D(x, y, z);
        return (idx < total_voxels) ? getBit(idx) : false;
    }
    
    /**
     * @brief Convert 3D coordinates to linear index
     * @param x X coordinate
     * @param y Y coordinate
     * @param z Z coordinate
     * @return Linear index, or total_voxels if out of bounds
     */
    inline uint64_t voxelIndex3Dto1D(int x, int y, int z) const {
        if (x < 0 || x >= grid_x || y < 0 || y >= grid_y || z < 0 || z >= grid_z) {
            return total_voxels; // Invalid index
        }
        return static_cast<uint64_t>(z) * grid_x * grid_y + 
               static_cast<uint64_t>(y) * grid_x + 
               static_cast<uint64_t>(x);
    }
    
    /**
     * @brief Get memory usage in bytes
     * @return Memory usage in bytes
     */
    size_t getMemoryUsageBytes() const {
        return bit_vector.size() * sizeof(uint64_t);
    }
    
    /**
     * @brief Get total number of voxels
     * @return Total voxel count
     */
    uint64_t getTotalVoxels() const {
        return total_voxels;
    }
    
    /**
     * @brief Count the number of occupied voxels
     * @return Number of bits set to 1
     */
    size_t countOccupiedVoxels() const {
        size_t count = 0;
        for (uint64_t word : bit_vector) {
            count += __builtin_popcountll(word);
        }
        return count;
    }
    
    /**
     * @brief Clear all voxels
     */
    void clear() {
        std::fill(bit_vector.begin(), bit_vector.end(), 0);
    }
    
    /**
     * @brief Get grid dimensions
     */
    void getGridDimensions(int& x, int& y, int& z) const {
        x = grid_x;
        y = grid_y;
        z = grid_z;
    }
    
    /**
     * @brief Check if coordinates are valid
     */
    inline bool isValidCoordinate(int x, int y, int z) const {
        return x >= 0 && x < grid_x && 
               y >= 0 && y < grid_y && 
               z >= 0 && z < grid_z;
    }
};

} // namespace pointcloud_compressor

#endif // POINTCLOUD_COMPRESSOR_BIT_VOXEL_GRID_HPP