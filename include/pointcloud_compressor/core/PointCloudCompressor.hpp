#ifndef POINTCLOUD_COMPRESSOR_POINT_CLOUD_COMPRESSOR_HPP
#define POINTCLOUD_COMPRESSOR_POINT_CLOUD_COMPRESSOR_HPP

#include <string>
#include <memory>
#include <optional>
#include <map>
#include "pointcloud_compressor/io/PointCloudIO.hpp"
#include "pointcloud_compressor/core/VoxelProcessor.hpp"
#include "pointcloud_compressor/core/PatternDictionaryBuilder.hpp"
#include "pointcloud_compressor/core/PatternEncoder.hpp"

namespace pointcloud_compressor {

struct CompressionSettings {
    float voxel_size = 0.01f;
    int block_size = 8;
    bool use_8bit_indices = false;  // Deprecated, kept for compatibility
    int index_bit_size = 0;  // 0=auto, 8, 16, 32, 64
    int min_points_threshold = 1;
    std::string temp_directory = "temp";
    float bounding_box_margin_ratio = 0.0f;

    CompressionSettings() = default;
    CompressionSettings(float vs, int bs, bool use8bit = false, int min_pts = 1, float margin_ratio = 0.2f)
        : voxel_size(vs), block_size(bs), use_8bit_indices(use8bit),
          index_bit_size(use8bit ? 8 : 0), min_points_threshold(min_pts),
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
        double save_ms = 0.0;
        double total_ms = 0.0;
    } timings;

    // Additional data for ROS message generation
    std::vector<uint64_t> block_indices;  // Store as largest type, actual size determined by index_bit_size
    VoxelGrid voxel_grid;
    std::vector<std::vector<uint8_t>> pattern_dictionary;
    uint64_t max_index = 0;  // Max index value for efficient encoding determination
    int index_bit_size = 16;  // Actual bit size used: 8, 16, 32, or 64
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
    std::map<int, float> tested_results;  // block_size -> compression_ratio
    double optimization_time_ms = 0.0;
};

class PointCloudCompressor {
public:
    // Constructor
    PointCloudCompressor(const CompressionSettings& settings = CompressionSettings());

    // Destructor
    ~PointCloudCompressor();

    // Main compression function
    CompressionResult compress(const std::string& input_file,
                              const std::string& output_prefix);

    // Main decompression function
    bool decompress(const std::string& compressed_prefix,
                   const std::string& output_file);

    // Decompress to VoxelGrid (preserves exact grid structure)
    bool decompressToGrid(const std::string& compressed_prefix,
                         VoxelGrid& grid);

    // Find optimal compression settings
    CompressionSettings findOptimalSettings(const std::string& input_file,
                                           float min_voxel_size = 0.005f,
                                           float max_voxel_size = 0.05f);

    // Find optimal block size
    BlockSizeOptimizationResult findOptimalBlockSize(
        const std::string& input_file,
        int min_block_size = 4,
        int max_block_size = 32,
        int step_size = 1,
        bool verbose = false);

    // Update settings
    void updateSettings(const CompressionSettings& settings);
    CompressionSettings getSettings() const;

    // Utility functions
    bool validateInputFile(const std::string& filename);
    size_t estimateMemoryUsage(const std::string& input_file);

    // Voxel grid caching
    std::optional<VoxelGrid> getCachedVoxelGrid() const;
    void clearCachedVoxelGrid();

private:
    CompressionSettings settings_;
    std::unique_ptr<VoxelProcessor> voxel_processor_;
    std::unique_ptr<PatternDictionaryBuilder> dictionary_builder_;
    std::unique_ptr<PatternEncoder> pattern_encoder_;

    // Cached voxel grid for reuse
    mutable std::optional<VoxelGrid> cached_voxel_grid_;

    // Internal compression steps
    bool loadPointCloud(const std::string& filename, PointCloud& cloud);
    bool voxelizeAndDivide(const PointCloud& cloud, std::vector<VoxelBlock>& blocks);
    bool voxelizeAndDivideWithGrid(const PointCloud& cloud,
                                   std::vector<VoxelBlock>& blocks,
                                   VoxelGrid& grid);
    bool buildDictionaryAndEncode(const std::vector<VoxelBlock>& blocks,
                                 std::vector<uint64_t>& indices);
    bool saveCompressionData(const std::string& output_prefix,
                           const std::vector<uint64_t>& indices,
                           const VoxelGrid& grid,
                           int index_bit_size = 16);

    // Internal decompression steps
    bool loadCompressionData(const std::string& compressed_prefix,
                           std::vector<uint64_t>& indices,
                           VoxelGrid& grid);
    bool reconstructPointCloud(const std::vector<uint64_t>& indices,
                             const VoxelGrid& grid,
                             PointCloud& cloud);
    bool reconstructVoxelGrid(const std::vector<uint64_t>& indices,
                            const VoxelGrid& metadata_grid,
                            VoxelGrid& reconstructed_grid);

    // File management
    void initializeTempPaths(const std::string& output_prefix);
    void cleanupTempFiles();

    std::string getBlocksFilename(const std::string& prefix) const;
    std::string getDictionaryFilename(const std::string& prefix) const;
    std::string getIndicesFilename(const std::string& prefix) const;
    std::string getMetadataFilename(const std::string& prefix) const;
};

} // namespace pointcloud_compressor

#endif // POINTCLOUD_COMPRESSOR_POINT_CLOUD_COMPRESSOR_HPP
