#ifndef POINTCLOUD_COMPRESSOR_VOXEL_PROCESSOR_HPP
#define POINTCLOUD_COMPRESSOR_VOXEL_PROCESSOR_HPP

#include <vector>
#include <memory>
#include <string>
#include "pointcloud_compressor/io/PcdIO.hpp"
#include "pointcloud_compressor/model/VoxelGrid.hpp"

namespace pointcloud_compressor {

// Forward declarations
class VoxelGrid;
class VoxelBlock;

class VoxelProcessor {
public:
    // Constructor
    VoxelProcessor(float voxel_size = 0.01f, int block_size = 8, int min_points_threshold = 1);
    
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
    
private:
    float voxel_size_;  // Size of each voxel
    int block_size_;    // Size of each block (e.g., 8x8x8)
    int min_points_threshold_;  // Minimum points per voxel to mark as occupied
    
    // Helper functions
    void computeBoundingBox(const PointCloud& cloud, 
                          Point3D& min_pt, Point3D& max_pt);
    
    int pointToVoxelIndex(const Point3D& point, const Point3D& min_pt, 
                         int& x, int& y, int& z, 
                         int grid_x, int grid_y, int grid_z);
    
    Point3D voxelToPoint(int x, int y, int z, const Point3D& min_pt);
};

} // namespace pointcloud_compressor

#endif // POINTCLOUD_COMPRESSOR_VOXEL_PROCESSOR_HPP