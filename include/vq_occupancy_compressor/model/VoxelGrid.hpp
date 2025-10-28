// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_VOXEL_GRID_HPP
#define VQ_OCCUPANCY_COMPRESSOR_VOXEL_GRID_HPP

#include <vector>
#include <memory>
#include <bitset>
#include <cstdint>
#include <unordered_map>

namespace vq_occupancy_compressor {

// 3D integer coordinates
struct VoxelCoord {
    int x, y, z;
    
    VoxelCoord() : x(0), y(0), z(0) {}
    VoxelCoord(int x_, int y_, int z_) : x(x_), y(y_), z(z_) {}
};

// Voxel block representing a cubic region
class VoxelBlock {
public:
    VoxelBlock(int size = 8);
    ~VoxelBlock();
    
    // Set/get voxel value
    void setVoxel(int x, int y, int z, bool occupied);
    bool getVoxel(int x, int y, int z) const;
    
    // Get block properties
    int getSize() const { return size; }
    int getOccupiedCount() const;
    bool isEmpty() const;
    
    // Convert to/from byte pattern
    std::vector<uint8_t> toBytePattern() const;
    void fromBytePattern(const std::vector<uint8_t>& pattern);
    
    int size;
    VoxelCoord position;  // Block position within the parent grid
    
private:
    std::vector<uint8_t> voxels_;  // Flattened 3D array (using uint8_t to avoid std::vector<bool> issues)
    
    int indexFromCoord(int x, int y, int z) const;
};

// 3D voxel grid
class VoxelGrid {
public:
    VoxelGrid();
    VoxelGrid(int dim_x, int dim_y, int dim_z, float voxel_size = 0.01f);
    ~VoxelGrid();
    
    // Initialize grid with dimensions
    void initialize(int dim_x, int dim_y, int dim_z, float voxel_size = 0.01f);
    
    // Set/get voxel value
    void setVoxel(int x, int y, int z, bool occupied);
    bool getVoxel(int x, int y, int z) const;
    
    // Get dimensions
    VoxelCoord getDimensions() const;
    void setDimensions(int x, int y, int z);
    
    // Get statistics
    int getOccupiedVoxelCount() const;
    int getTotalVoxelCount() const;
    float getOccupancyRatio() const;
    
    // Clear grid
    void clear();
    
    // Check if coordinates are valid
    bool isValidCoord(int x, int y, int z) const;
    
    // Get/set voxel size
    float getVoxelSize() const { return voxel_size_; }
    void setVoxelSize(float size) { voxel_size_ = size; }
    
    // Get/set origin
    void setOrigin(float x, float y, float z);
    void getOrigin(float& x, float& y, float& z) const;
    
    // Extract a block from the grid
    VoxelBlock extractBlock(int block_x, int block_y, int block_z, int block_size) const;
    
    // Insert a block into the grid
    void insertBlock(const VoxelBlock& block, int block_x, int block_y, int block_z);
    
private:
    int dim_x_, dim_y_, dim_z_;  // Grid dimensions
    float voxel_size_;            // Size of each voxel
    float origin_x_, origin_y_, origin_z_;  // Grid origin
    
    // Use sparse representation for large grids
    static constexpr uint64_t SPARSE_THRESHOLD = 100000000; // 100M voxels
    bool use_sparse_;
    std::vector<uint8_t> voxels_;   // Dense array for small grids
    std::unordered_map<uint64_t, bool> sparse_voxels_; // Sparse map for large grids
    
    uint64_t indexFromCoord(int x, int y, int z) const;
};

} // namespace vq_occupancy_compressor

#endif // VQ_OCCUPANCY_COMPRESSOR_VOXEL_GRID_HPP
