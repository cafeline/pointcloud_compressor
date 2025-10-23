// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_VOXEL_PROCESSOR_HPP
#define POINTCLOUD_COMPRESSOR_VOXEL_PROCESSOR_HPP

#include <vector>
#include <memory>
#include <string>
#include <cstdint>
#include "pointcloud_compressor/io/PcdIO.hpp"
#include "pointcloud_compressor/model/VoxelGrid.hpp"

namespace pointcloud_compressor {

// Forward declarations
class VoxelGrid;
class VoxelBlock;

struct VoxelizationReport {
    int grid_x = 0;
    int grid_y = 0;
    int grid_z = 0;
    uint64_t total_voxels = 0;
    int occupied_voxels_estimate = 0;
    int occupied_voxels_committed = 0;
    double bbox_time_ms = 0.0;
    double grid_init_time_ms = 0.0;
    double voxel_count_time_ms = 0.0;
    double grid_transfer_time_ms = 0.0;
    double voxelization_time_ms = 0.0;
    int blocks_x = 0;
    int blocks_y = 0;
    int blocks_z = 0;
    int total_blocks = 0;
    double block_division_time_ms = 0.0;
};

class VoxelProcessor {
public:
    // Constructor
    VoxelProcessor(float voxel_size = 0.01f, int block_size = 8, int min_points_threshold = 1,
                   float bounding_box_margin_ratio = 0.0f);
    
    // Destructor
    ~VoxelProcessor();
    
    // Voxelize point cloud into 3D grid
    bool voxelizePointCloud(const PointCloud& cloud, VoxelGrid& grid);
    
    // Divide voxel grid into blocks
    bool divideIntoBlocks(const VoxelGrid& grid, std::vector<VoxelBlock>& blocks);
    
    // Calculate block count
    void calculateBlockCount(const VoxelGrid& grid, 
                           int& x_blocks, int& y_blocks, int& z_blocks, 
                           int& total_blocks);
    
    // Extract pattern from a block
    std::vector<uint8_t> extractPattern(const VoxelBlock& block);
    
    // Reconstruct point cloud from voxel grid
    bool reconstructPointCloud(const VoxelGrid& grid, PointCloud& cloud);
    
    // Find optimal voxel size
    float findOptimalVoxelSize(const PointCloud& cloud, 
                               float min_size = 0.005f, 
                               float max_size = 0.05f, 
                               float step = 0.005f);
    
    // Calculate pattern size
    void calculatePatternSize(int& pattern_bits, int& pattern_bytes);
    
    // Save blocks to file
    bool saveBlocksToFile(const std::vector<VoxelBlock>& blocks, 
                         const std::string& filename);
    
    // Load blocks from file
    bool loadBlocksFromFile(const std::string& filename, 
                           std::vector<VoxelBlock>& blocks);
    
    // Getters and setters
    float getVoxelSize() const { return voxel_size_; }
    void setVoxelSize(float size) { voxel_size_ = size; }
    
    int getBlockSize() const { return block_size_; }
    void setBlockSize(int size) { block_size_ = size; }
    
    int getMinPointsThreshold() const { return min_points_threshold_; }
    void setMinPointsThreshold(int threshold) { min_points_threshold_ = std::max(1, threshold); }
    void setBoundingBoxMarginRatio(float ratio) { bounding_box_margin_ratio_ = std::max(0.0f, ratio); }
    float getBoundingBoxMarginRatio() const { return bounding_box_margin_ratio_; }

    const VoxelizationReport& getLastReport() const { return last_report_; }

private:
    float voxel_size_;  // Size of each voxel
    int block_size_;    // Size of each block (e.g., 8x8x8)
    int min_points_threshold_;  // Minimum points per voxel to mark as occupied
    float bounding_box_margin_ratio_;
    VoxelizationReport last_report_{};

    // Helper functions
    void computeBoundingBox(const PointCloud& cloud, 
                          Point3D& min_pt, Point3D& max_pt);
    
    int pointToVoxelIndex(const Point3D& point, const Point3D& min_pt, 
                         int& x, int& y, int& z, 
                         int grid_x, int grid_y, int grid_z);
    
    Point3D voxelToPoint(int x, int y, int z, const Point3D& min_pt);
    
    // Inline function for performance-critical operations
    inline void pointToVoxelIndexFast(const Point3D& point, const Point3D& min_pt,
                                      int& x, int& y, int& z) const {
        x = static_cast<int>((point.x - min_pt.x) / voxel_size_);
        y = static_cast<int>((point.y - min_pt.y) / voxel_size_);
        z = static_cast<int>((point.z - min_pt.z) / voxel_size_);
    }
};

} // namespace pointcloud_compressor

#endif // POINTCLOUD_COMPRESSOR_VOXEL_PROCESSOR_HPP
