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


struct VoxelCoord {
    int x, y, z;
    
    VoxelCoord() : x(0), y(0), z(0) {}
    VoxelCoord(int x_, int y_, int z_) : x(x_), y(y_), z(z_) {}
};


class VoxelBlock {
public:
    VoxelBlock(int size = 8);
    ~VoxelBlock();
    
    
    void setVoxel(int x, int y, int z, bool occupied);
    bool getVoxel(int x, int y, int z) const;
    
    
    int getSize() const { return size; }
    int getOccupiedCount() const;
    bool isEmpty() const;
    
    
    std::vector<uint8_t> toBytePattern() const;
    void fromBytePattern(const std::vector<uint8_t>& pattern);
    
    int size;
    VoxelCoord position;  
    
private:
    std::vector<uint8_t> voxels_;  
    
    int indexFromCoord(int x, int y, int z) const;
};


class VoxelGrid {
public:
    VoxelGrid();
    VoxelGrid(int dim_x, int dim_y, int dim_z, float voxel_size = 0.01f);
    ~VoxelGrid();
    
    
    void initialize(int dim_x, int dim_y, int dim_z, float voxel_size = 0.01f);
    
    
    void setVoxel(int x, int y, int z, bool occupied);
    bool getVoxel(int x, int y, int z) const;
    
    
    VoxelCoord getDimensions() const;
    void setDimensions(int x, int y, int z);
    
    
    int getOccupiedVoxelCount() const;
    int getTotalVoxelCount() const;
    float getOccupancyRatio() const;
    
    
    void clear();
    
    
    bool isValidCoord(int x, int y, int z) const;
    
    
    float getVoxelSize() const { return voxel_size_; }
    void setVoxelSize(float size) { voxel_size_ = size; }
    
    
    void setOrigin(float x, float y, float z);
    void getOrigin(float& x, float& y, float& z) const;
    
    
    VoxelBlock extractBlock(int block_x, int block_y, int block_z, int block_size) const;
    
    
    void insertBlock(const VoxelBlock& block, int block_x, int block_y, int block_z);
    
private:
    int dim_x_, dim_y_, dim_z_;  
    float voxel_size_;            
    float origin_x_, origin_y_, origin_z_;  
    
    
    static constexpr uint64_t SPARSE_THRESHOLD = 100000000; 
    bool use_sparse_;
    std::vector<uint8_t> voxels_;   
    std::unordered_map<uint64_t, bool> sparse_voxels_; 
    
    uint64_t indexFromCoord(int x, int y, int z) const;
};

} 

#endif 
