#include "pointcloud_compressor/core/VoxelProcessor.hpp"
#include "pointcloud_compressor/model/VoxelGrid.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <fstream>
#include <iostream>
#include <map>
#include <tuple>

namespace pointcloud_compressor {

VoxelProcessor::VoxelProcessor(float voxel_size, int block_size, int min_points_threshold)
    : voxel_size_(voxel_size), block_size_(block_size), 
      min_points_threshold_(std::max(1, min_points_threshold)) {}

VoxelProcessor::~VoxelProcessor() {}

bool VoxelProcessor::voxelizePointCloud(const PointCloud& cloud, VoxelGrid& grid) {
    if (cloud.empty()) {
        return false;
    }
    
    // Compute bounding box
    Point3D min_pt, max_pt;
    computeBoundingBox(cloud, min_pt, max_pt);
    
    // Calculate grid dimensions
    int grid_x = static_cast<int>(std::ceil((max_pt.x - min_pt.x) / voxel_size_)) + 1;
    int grid_y = static_cast<int>(std::ceil((max_pt.y - min_pt.y) / voxel_size_)) + 1;
    int grid_z = static_cast<int>(std::ceil((max_pt.z - min_pt.z) / voxel_size_)) + 1;
    
    // Initialize grid
    grid.initialize(grid_x, grid_y, grid_z, voxel_size_);
    grid.setOrigin(min_pt.x, min_pt.y, min_pt.z);
    
    // Count points per voxel
    std::map<std::tuple<int, int, int>, int> voxel_point_count;
    
    for (const auto& point : cloud.points) {
        int x, y, z;
        if (pointToVoxelIndex(point, min_pt, x, y, z, grid_x, grid_y, grid_z) >= 0) {
            auto voxel_key = std::make_tuple(x, y, z);
            voxel_point_count[voxel_key]++;
        }
    }
    
    // Debug: Log filtering statistics
    int total_voxels = voxel_point_count.size();
    int occupied_voxels = 0;
    int filtered_voxels = 0;
    
    // Set voxel occupancy based on point count threshold
    for (const auto& entry : voxel_point_count) {
        int x = std::get<0>(entry.first);
        int y = std::get<1>(entry.first);
        int z = std::get<2>(entry.first);
        int point_count = entry.second;
        
        if (point_count >= min_points_threshold_) {
            grid.setVoxel(x, y, z, true);
            occupied_voxels++;
        } else {
            filtered_voxels++;
        }
    }
    
    // Debug output
    std::cout << "[VoxelProcessor] min_points_threshold=" << min_points_threshold_ << std::endl;
    std::cout << "[VoxelProcessor] Total voxels with points: " << total_voxels << std::endl;
    std::cout << "[VoxelProcessor] Occupied voxels (>= threshold): " << occupied_voxels << std::endl;
    std::cout << "[VoxelProcessor] Filtered voxels (< threshold): " << filtered_voxels << std::endl;
    
    return true;
}

bool VoxelProcessor::divideIntoBlocks(const VoxelGrid& grid, std::vector<VoxelBlock>& blocks) {
    blocks.clear();
    
    VoxelCoord dims = grid.getDimensions();
    
    // Calculate number of blocks in each dimension
    int x_blocks = (dims.x + block_size_ - 1) / block_size_;
    int y_blocks = (dims.y + block_size_ - 1) / block_size_;
    int z_blocks = (dims.z + block_size_ - 1) / block_size_;
    
    // Extract blocks
    for (int bz = 0; bz < z_blocks; ++bz) {
        for (int by = 0; by < y_blocks; ++by) {
            for (int bx = 0; bx < x_blocks; ++bx) {
                VoxelBlock block = grid.extractBlock(bx, by, bz, block_size_);
                blocks.push_back(block);
            }
        }
    }
    
    return !blocks.empty();
}

void VoxelProcessor::calculateBlockCount(const VoxelGrid& grid, 
                                       int& x_blocks, int& y_blocks, int& z_blocks, 
                                       int& total_blocks) {
    VoxelCoord dims = grid.getDimensions();
    
    x_blocks = (dims.x + block_size_ - 1) / block_size_;
    y_blocks = (dims.y + block_size_ - 1) / block_size_;
    z_blocks = (dims.z + block_size_ - 1) / block_size_;
    total_blocks = x_blocks * y_blocks * z_blocks;
}

std::vector<uint8_t> VoxelProcessor::extractPattern(const VoxelBlock& block) {
    return block.toBytePattern();
}

bool VoxelProcessor::reconstructPointCloud(const VoxelGrid& grid, PointCloud& cloud) {
    cloud.clear();
    
    VoxelCoord dims = grid.getDimensions();
    float origin_x, origin_y, origin_z;
    grid.getOrigin(origin_x, origin_y, origin_z);
    
    Point3D min_pt(origin_x, origin_y, origin_z);
    
    // Extract occupied voxels as points
    for (int z = 0; z < dims.z; ++z) {
        for (int y = 0; y < dims.y; ++y) {
            for (int x = 0; x < dims.x; ++x) {
                if (grid.getVoxel(x, y, z)) {
                    Point3D point = voxelToPoint(x, y, z, min_pt);
                    cloud.points.push_back(point);
                }
            }
        }
    }
    
    return !cloud.empty();
}

float VoxelProcessor::findOptimalVoxelSize(const PointCloud& cloud, 
                                         float min_size, float max_size, float step) {
    if (cloud.empty()) {
        return min_size;
    }
    
    float best_size = min_size;
    float best_score = 0.0f;
    
    // Try different voxel sizes
    for (float size = min_size; size <= max_size; size += step) {
        VoxelProcessor temp_processor(size, block_size_, min_points_threshold_);
        VoxelGrid temp_grid;
        
        if (temp_processor.voxelizePointCloud(cloud, temp_grid)) {
            // Score based on occupancy ratio and compression potential
            float occupancy = temp_grid.getOccupancyRatio();
            float compression_potential = 1.0f / (1.0f + occupancy);
            float score = occupancy * compression_potential;
            
            if (score > best_score) {
                best_score = score;
                best_size = size;
            }
        }
    }
    
    return best_size;
}

void VoxelProcessor::calculatePatternSize(int& pattern_bits, int& pattern_bytes) {
    pattern_bits = block_size_ * block_size_ * block_size_;
    pattern_bytes = (pattern_bits + 7) / 8;
}

bool VoxelProcessor::saveBlocksToFile(const std::vector<VoxelBlock>& blocks, 
                                    const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    // Write header
    uint32_t num_blocks = static_cast<uint32_t>(blocks.size());
    uint32_t block_size = static_cast<uint32_t>(block_size_);
    
    file.write(reinterpret_cast<const char*>(&num_blocks), sizeof(num_blocks));
    file.write(reinterpret_cast<const char*>(&block_size), sizeof(block_size));
    
    // Write blocks
    for (const auto& block : blocks) {
        // Write position
        file.write(reinterpret_cast<const char*>(&block.position.x), sizeof(int));
        file.write(reinterpret_cast<const char*>(&block.position.y), sizeof(int));
        file.write(reinterpret_cast<const char*>(&block.position.z), sizeof(int));
        
        // Write pattern
        std::vector<uint8_t> pattern = block.toBytePattern();
        uint32_t pattern_size = static_cast<uint32_t>(pattern.size());
        file.write(reinterpret_cast<const char*>(&pattern_size), sizeof(pattern_size));
        file.write(reinterpret_cast<const char*>(pattern.data()), pattern_size);
    }
    
    return file.good();
}

bool VoxelProcessor::loadBlocksFromFile(const std::string& filename, 
                                      std::vector<VoxelBlock>& blocks) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    blocks.clear();
    
    // Read header
    uint32_t num_blocks, block_size;
    file.read(reinterpret_cast<char*>(&num_blocks), sizeof(num_blocks));
    file.read(reinterpret_cast<char*>(&block_size), sizeof(block_size));
    
    if (!file.good()) {
        return false;
    }
    
    blocks.reserve(num_blocks);
    
    // Read blocks
    for (uint32_t i = 0; i < num_blocks; ++i) {
        VoxelBlock block(block_size);
        
        // Read position
        file.read(reinterpret_cast<char*>(&block.position.x), sizeof(int));
        file.read(reinterpret_cast<char*>(&block.position.y), sizeof(int));
        file.read(reinterpret_cast<char*>(&block.position.z), sizeof(int));
        
        // Read pattern
        uint32_t pattern_size;
        file.read(reinterpret_cast<char*>(&pattern_size), sizeof(pattern_size));
        
        std::vector<uint8_t> pattern(pattern_size);
        file.read(reinterpret_cast<char*>(pattern.data()), pattern_size);
        
        if (!file.good()) {
            return false;
        }
        
        block.fromBytePattern(pattern);
        blocks.push_back(block);
    }
    
    return true;
}

// Private helper functions
void VoxelProcessor::computeBoundingBox(const PointCloud& cloud, 
                                      Point3D& min_pt, Point3D& max_pt) {
    if (cloud.empty()) {
        min_pt = max_pt = Point3D(0, 0, 0);
        return;
    }
    
    min_pt = max_pt = cloud.points[0];
    
    for (const auto& point : cloud.points) {
        min_pt.x = std::min(min_pt.x, point.x);
        min_pt.y = std::min(min_pt.y, point.y);
        min_pt.z = std::min(min_pt.z, point.z);
        
        max_pt.x = std::max(max_pt.x, point.x);
        max_pt.y = std::max(max_pt.y, point.y);
        max_pt.z = std::max(max_pt.z, point.z);
    }
}

int VoxelProcessor::pointToVoxelIndex(const Point3D& point, const Point3D& min_pt, 
                                    int& x, int& y, int& z, 
                                    int grid_x, int grid_y, int grid_z) {
    x = static_cast<int>((point.x - min_pt.x) / voxel_size_);
    y = static_cast<int>((point.y - min_pt.y) / voxel_size_);
    z = static_cast<int>((point.z - min_pt.z) / voxel_size_);
    
    // Clamp to grid bounds
    x = std::max(0, std::min(x, grid_x - 1));
    y = std::max(0, std::min(y, grid_y - 1));
    z = std::max(0, std::min(z, grid_z - 1));
    
    return z * grid_x * grid_y + y * grid_x + x;
}

Point3D VoxelProcessor::voxelToPoint(int x, int y, int z, const Point3D& min_pt) {
    // Return center of voxel
    return Point3D(
        min_pt.x + (x + 0.5f) * voxel_size_,
        min_pt.y + (y + 0.5f) * voxel_size_,
        min_pt.z + (z + 0.5f) * voxel_size_
    );
}

} // namespace pointcloud_compressor