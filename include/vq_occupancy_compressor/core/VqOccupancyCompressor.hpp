// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_VQ_OCCUPANCY_COMPRESSOR_HPP
#define VQ_OCCUPANCY_COMPRESSOR_VQ_OCCUPANCY_COMPRESSOR_HPP

#include <string>
#include <memory>
#include <optional>
#include <map>
#include "vq_occupancy_compressor/io/PointCloudIO.hpp"
#include "vq_occupancy_compressor/core/VoxelProcessor.hpp"
#include "vq_occupancy_compressor/core/PatternDictionaryBuilder.hpp"

namespace vq_occupancy_compressor {

struct CompressionSettings {
    float voxel_size = 0.01f;
    int block_size = 8;
    int min_points_threshold = 1;
    float bounding_box_margin_ratio = 0.0f;

    CompressionSettings() = default;
    CompressionSettings(float vs, int bs, bool  = false, int min_pts = 1, float margin_ratio = 0.2f)
        : voxel_size(vs), block_size(bs), min_points_threshold(min_pts),
          bounding_box_margin_ratio(margin_ratio) {}
};

struct CompressionResult {
    bool success = false;
    float compression_ratio = 1.0f;
    size_t original_size = 0;
    size_t compressed_size = 0;
    size_t num_blocks = 0;
    size_t num_unique_patterns = 0;
    std::string error_message;
    size_t point_count = 0;
    std::string input_file;
    float voxel_size = 0.0f;
    int block_size = 0;
    struct StageTimings {
        double load_ms = 0.0;
        double voxelize_ms = 0.0;
        double dictionary_ms = 0.0;
        double total_ms = 0.0;
    } timings;

    
    std::vector<uint64_t> block_indices;  
    VoxelGrid voxel_grid;
    std::vector<std::vector<uint8_t>> pattern_dictionary;
    uint64_t max_index = 0;  
    int index_bit_size = 16;  
    struct {
        double x, y, z;
    } grid_dimensions;
    struct {
        double x, y, z;
    } grid_origin;
    struct {
        int x, y, z;
    } blocks_count;
    struct {
        double x, y, z;
    } margin{0.0, 0.0, 0.0};
    struct {
        int grid_x = 0;
        int grid_y = 0;
        int grid_z = 0;
        uint64_t total_voxels = 0;
        int occupied_voxels = 0;
        int transferred_voxels = 0;
        int block_count = 0;
        struct {
            int x = 0;
            int y = 0;
            int z = 0;
        } blocks_per_axis;
        double bbox_time_ms = 0.0;
        double grid_init_time_ms = 0.0;
        double voxel_count_time_ms = 0.0;
        double grid_transfer_time_ms = 0.0;
        double voxelization_time_ms = 0.0;
        double block_division_time_ms = 0.0;
    } voxel_stats;

    std::string formatDetailedReport() const;
};

struct BlockSizeOptimizationResult {
    int optimal_block_size = -1;
    float best_compression_ratio = 1.0f;
    std::map<int, float> tested_results;  
    double optimization_time_ms = 0.0;
};

class VqOccupancyCompressor {
public:
    
    VqOccupancyCompressor(const CompressionSettings& settings = CompressionSettings());

    
    ~VqOccupancyCompressor();

    
    CompressionResult compress(const std::string& input_file);

    
    CompressionSettings findOptimalSettings(const std::string& input_file,
                                           float min_voxel_size = 0.005f,
                                           float max_voxel_size = 0.05f);

    
    BlockSizeOptimizationResult findOptimalBlockSize(
        const std::string& input_file,
        int min_block_size = 4,
        int max_block_size = 32,
        int step_size = 1,
        bool verbose = false);

    
    void updateSettings(const CompressionSettings& settings);
    CompressionSettings getSettings() const;

    
    bool validateInputFile(const std::string& filename);
    size_t estimateMemoryUsage(const std::string& input_file);

    
    std::optional<VoxelGrid> getCachedVoxelGrid() const;
    void clearCachedVoxelGrid();

private:
    CompressionSettings settings_;
    std::unique_ptr<VoxelProcessor> voxel_processor_;
    std::unique_ptr<PatternDictionaryBuilder> dictionary_builder_;

    
    mutable std::optional<VoxelGrid> cached_voxel_grid_;

    
    bool loadPointCloud(const std::string& filename, PointCloud& cloud);
    bool voxelizeAndDivide(const PointCloud& cloud, std::vector<VoxelBlock>& blocks);
    bool voxelizeAndDivideWithGrid(const PointCloud& cloud,
                                   std::vector<VoxelBlock>& blocks,
                                   VoxelGrid& grid);
    bool buildDictionaryAndEncode(const std::vector<VoxelBlock>& blocks,
                                 std::vector<uint64_t>& indices);
};

} 

#endif 
