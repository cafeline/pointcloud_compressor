#include "pointcloud_compressor/model/VoxelGrid.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace pointcloud_compressor {

// VoxelBlock implementation
VoxelBlock::VoxelBlock(int size) : size(size), position(0, 0, 0) {
    int total_voxels = size * size * size;
    voxels_.resize(total_voxels, false);
}

VoxelBlock::~VoxelBlock() {}

void VoxelBlock::setVoxel(int x, int y, int z, bool occupied) {
    if (x >= 0 && x < size && y >= 0 && y < size && z >= 0 && z < size) {
        voxels_[indexFromCoord(x, y, z)] = occupied;
    }
}

bool VoxelBlock::getVoxel(int x, int y, int z) const {
    if (x >= 0 && x < size && y >= 0 && y < size && z >= 0 && z < size) {
        return voxels_[indexFromCoord(x, y, z)];
    }
    return false;
}

int VoxelBlock::getOccupiedCount() const {
    return std::count(voxels_.begin(), voxels_.end(), true);
}

bool VoxelBlock::isEmpty() const {
    return getOccupiedCount() == 0;
}

std::vector<uint8_t> VoxelBlock::toBytePattern() const {
    int total_bits = size * size * size;
    int total_bytes = (total_bits + 7) / 8;
    std::vector<uint8_t> pattern(total_bytes, 0);
    
    for (int i = 0; i < total_bits; ++i) {
        if (i < voxels_.size() && voxels_[i]) {
            int byte_index = i / 8;
            int bit_index = i % 8;
            pattern[byte_index] |= (1 << bit_index);
        }
    }
    
    return pattern;
}

void VoxelBlock::fromBytePattern(const std::vector<uint8_t>& pattern) {
    int total_bits = size * size * size;
    voxels_.assign(total_bits, false);
    
    for (int i = 0; i < total_bits && i / 8 < pattern.size(); ++i) {
        int byte_index = i / 8;
        int bit_index = i % 8;
        if (pattern[byte_index] & (1 << bit_index)) {
            voxels_[i] = true;
        }
    }
}

int VoxelBlock::indexFromCoord(int x, int y, int z) const {
    return z * size * size + y * size + x;
}

// VoxelGrid implementation
VoxelGrid::VoxelGrid() : dim_x_(0), dim_y_(0), dim_z_(0), voxel_size_(0.01f),
                         origin_x_(0), origin_y_(0), origin_z_(0) {}

VoxelGrid::VoxelGrid(int dim_x, int dim_y, int dim_z, float voxel_size)
    : voxel_size_(voxel_size), origin_x_(0), origin_y_(0), origin_z_(0) {
    initialize(dim_x, dim_y, dim_z, voxel_size);
}

VoxelGrid::~VoxelGrid() {}

void VoxelGrid::initialize(int dim_x, int dim_y, int dim_z, float voxel_size) {
    dim_x_ = dim_x;
    dim_y_ = dim_y;
    dim_z_ = dim_z;
    voxel_size_ = voxel_size;
    
    int total_voxels = dim_x * dim_y * dim_z;
    voxels_.assign(total_voxels, false);
}

void VoxelGrid::setVoxel(int x, int y, int z, bool occupied) {
    if (isValidCoord(x, y, z)) {
        voxels_[indexFromCoord(x, y, z)] = occupied;
    }
}

bool VoxelGrid::getVoxel(int x, int y, int z) const {
    if (isValidCoord(x, y, z)) {
        return voxels_[indexFromCoord(x, y, z)];
    }
    return false;
}

VoxelCoord VoxelGrid::getDimensions() const {
    return VoxelCoord(dim_x_, dim_y_, dim_z_);
}

void VoxelGrid::setDimensions(int x, int y, int z) {
    initialize(x, y, z, voxel_size_);
}

int VoxelGrid::getOccupiedVoxelCount() const {
    return std::count(voxels_.begin(), voxels_.end(), true);
}

int VoxelGrid::getTotalVoxelCount() const {
    return dim_x_ * dim_y_ * dim_z_;
}

float VoxelGrid::getOccupancyRatio() const {
    int total = getTotalVoxelCount();
    if (total == 0) return 0.0f;
    return static_cast<float>(getOccupiedVoxelCount()) / total;
}

void VoxelGrid::clear() {
    std::fill(voxels_.begin(), voxels_.end(), false);
}

bool VoxelGrid::isValidCoord(int x, int y, int z) const {
    return x >= 0 && x < dim_x_ && y >= 0 && y < dim_y_ && z >= 0 && z < dim_z_;
}

void VoxelGrid::setOrigin(float x, float y, float z) {
    origin_x_ = x;
    origin_y_ = y;
    origin_z_ = z;
}

void VoxelGrid::getOrigin(float& x, float& y, float& z) const {
    x = origin_x_;
    y = origin_y_;
    z = origin_z_;
}

VoxelBlock VoxelGrid::extractBlock(int block_x, int block_y, int block_z, int block_size) const {
    VoxelBlock block(block_size);
    block.position = VoxelCoord(block_x, block_y, block_z);
    
    int start_x = block_x * block_size;
    int start_y = block_y * block_size;
    int start_z = block_z * block_size;
    
    for (int z = 0; z < block_size; ++z) {
        for (int y = 0; y < block_size; ++y) {
            for (int x = 0; x < block_size; ++x) {
                int grid_x = start_x + x;
                int grid_y = start_y + y;
                int grid_z = start_z + z;
                
                bool occupied = isValidCoord(grid_x, grid_y, grid_z) && 
                               getVoxel(grid_x, grid_y, grid_z);
                block.setVoxel(x, y, z, occupied);
            }
        }
    }
    
    return block;
}

void VoxelGrid::insertBlock(const VoxelBlock& block, int block_x, int block_y, int block_z) {
    int start_x = block_x * block.getSize();
    int start_y = block_y * block.getSize();
    int start_z = block_z * block.getSize();
    
    for (int z = 0; z < block.getSize(); ++z) {
        for (int y = 0; y < block.getSize(); ++y) {
            for (int x = 0; x < block.getSize(); ++x) {
                int grid_x = start_x + x;
                int grid_y = start_y + y;
                int grid_z = start_z + z;
                
                if (isValidCoord(grid_x, grid_y, grid_z)) {
                    setVoxel(grid_x, grid_y, grid_z, block.getVoxel(x, y, z));
                }
            }
        }
    }
}

int VoxelGrid::indexFromCoord(int x, int y, int z) const {
    return z * dim_x_ * dim_y_ + y * dim_x_ + x;
}

} // namespace pointcloud_compressor