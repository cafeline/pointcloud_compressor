// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <vector>
#include <random>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/core/VoxelProcessor.hpp"
#include "pointcloud_compressor/io/PlyIO.hpp"
#include "pointcloud_compressor/io/PcdIO.hpp"

using namespace pointcloud_compressor;

class VoxelRoundTripTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_dir_ = "/tmp/voxel_round_trip_test";
        std::filesystem::create_directories(test_dir_);
    }
    
    void TearDown() override {
        if (std::filesystem::exists(test_dir_)) {
            std::filesystem::remove_all(test_dir_);
        }
    }
    
    // Create a simple test point cloud
    void createTestPointCloud(const std::string& filename, int num_points = 1000) {
        PointCloud cloud;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(-10.0f, 10.0f);
        
        for (int i = 0; i < num_points; ++i) {
            Point3D point;
            point.x = dis(gen);
            point.y = dis(gen);
            point.z = dis(gen);
            cloud.points.push_back(point);
        }
        
        PlyIO::writePlyFile(filename, cloud);
    }
    
    // Compare two voxel grids
    bool compareVoxelGrids(const VoxelGrid& grid1, const VoxelGrid& grid2, 
                          std::vector<std::string>& differences) {
        differences.clear();
        
        // Check dimensions
        VoxelCoord dims1 = grid1.getDimensions();
        VoxelCoord dims2 = grid2.getDimensions();
        
        if (dims1.x != dims2.x || dims1.y != dims2.y || dims1.z != dims2.z) {
            differences.push_back("Grid dimensions mismatch: (" + 
                std::to_string(dims1.x) + "," + std::to_string(dims1.y) + "," + 
                std::to_string(dims1.z) + ") vs (" +
                std::to_string(dims2.x) + "," + std::to_string(dims2.y) + "," + 
                std::to_string(dims2.z) + ")");
            return false;
        }
        
        // Check voxel size
        float size1 = grid1.getVoxelSize();
        float size2 = grid2.getVoxelSize();
        if (std::abs(size1 - size2) > 1e-6) {
            differences.push_back("Voxel size mismatch: " + 
                std::to_string(size1) + " vs " + std::to_string(size2));
            return false;
        }
        
        // Check origin
        float ox1, oy1, oz1, ox2, oy2, oz2;
        grid1.getOrigin(ox1, oy1, oz1);
        grid2.getOrigin(ox2, oy2, oz2);
        
        if (std::abs(ox1 - ox2) > 1e-6 || std::abs(oy1 - oy2) > 1e-6 || 
            std::abs(oz1 - oz2) > 1e-6) {
            differences.push_back("Origin mismatch: (" + 
                std::to_string(ox1) + "," + std::to_string(oy1) + "," + 
                std::to_string(oz1) + ") vs (" +
                std::to_string(ox2) + "," + std::to_string(oy2) + "," + 
                std::to_string(oz2) + ")");
            return false;
        }
        
        // Check each voxel
        int mismatch_count = 0;
        int total_voxels = dims1.x * dims1.y * dims1.z;
        
        for (int z = 0; z < dims1.z; ++z) {
            for (int y = 0; y < dims1.y; ++y) {
                for (int x = 0; x < dims1.x; ++x) {
                    bool occupied1 = grid1.getVoxel(x, y, z);
                    bool occupied2 = grid2.getVoxel(x, y, z);
                    
                    if (occupied1 != occupied2) {
                        if (mismatch_count < 10) { // Report first 10 mismatches
                            differences.push_back("Voxel mismatch at (" + 
                                std::to_string(x) + "," + std::to_string(y) + "," + 
                                std::to_string(z) + "): " +
                                (occupied1 ? "occupied" : "empty") + " vs " +
                                (occupied2 ? "occupied" : "empty"));
                        }
                        mismatch_count++;
                    }
                }
            }
        }
        
        if (mismatch_count > 0) {
            differences.push_back("Total voxel mismatches: " + 
                std::to_string(mismatch_count) + " out of " + 
                std::to_string(total_voxels) + " (" +
                std::to_string(100.0 * mismatch_count / total_voxels) + "%)");
            
            // Count occupied voxels in each grid
            int occupied1 = grid1.getOccupiedVoxelCount();
            int occupied2 = grid2.getOccupiedVoxelCount();
            differences.push_back("Occupied voxel count: " + 
                std::to_string(occupied1) + " vs " + std::to_string(occupied2));
        }
        
        return mismatch_count == 0;
    }
    
    std::string test_dir_;
};

// Test basic round trip with small point cloud
TEST_F(VoxelRoundTripTest, BasicRoundTrip) {
    std::string input_file = test_dir_ + "/test.ply";
    std::string compressed_prefix = test_dir_ + "/compressed";
    std::string decompressed_file = test_dir_ + "/decompressed.ply";
    
    // Create test point cloud
    createTestPointCloud(input_file, 100);
    
    // Set up compressor with specific settings
    CompressionSettings settings;
    settings.voxel_size = 0.5f;
    settings.block_size = 8;
    settings.min_points_threshold = 1;
    
    PointCloudCompressor compressor(settings);
    
    // Compress
    auto result = compressor.compress(input_file, compressed_prefix);
    ASSERT_TRUE(result.success) << "Compression failed: " << result.error_message;
    
    std::cout << "Compression stats:" << std::endl;
    std::cout << "  Original size: " << result.original_size << " bytes" << std::endl;
    std::cout << "  Compressed size: " << result.compressed_size << " bytes" << std::endl;
    std::cout << "  Compression ratio: " << result.compression_ratio << std::endl;
    std::cout << "  Unique patterns: " << result.num_unique_patterns << std::endl;
    std::cout << "  Index bit size: " << result.index_bit_size << " bits" << std::endl;
    
    // Get original voxel grid
    auto original_grid = compressor.getCachedVoxelGrid();
    ASSERT_TRUE(original_grid.has_value()) << "Failed to get cached voxel grid";
    
    // Decompress
    PointCloudCompressor decompressor(settings);
    bool decompress_success = decompressor.decompress(compressed_prefix, decompressed_file);
    ASSERT_TRUE(decompress_success) << "Decompression failed";
    
    // Load decompressed point cloud and voxelize it
    PointCloud decompressed_cloud;
    ASSERT_TRUE(PlyIO::readPlyFile(decompressed_file, decompressed_cloud));
    
    // Voxelize the decompressed cloud with same settings
    VoxelProcessor voxel_processor(settings.voxel_size, settings.block_size, 
                                   settings.min_points_threshold);
    VoxelGrid reconstructed_grid;
    ASSERT_TRUE(voxel_processor.voxelizePointCloud(decompressed_cloud, reconstructed_grid));
    
    // Compare voxel grids
    std::vector<std::string> differences;
    bool grids_match = compareVoxelGrids(original_grid.value(), reconstructed_grid, differences);
    
    if (!grids_match) {
        std::cout << "\nVoxel grid differences found:" << std::endl;
        for (const auto& diff : differences) {
            std::cout << "  - " << diff << std::endl;
        }
    }
    
    EXPECT_TRUE(grids_match) << "Voxel grids do not match after round trip";
}

// Test with different block sizes
TEST_F(VoxelRoundTripTest, DifferentBlockSizes) {
    std::string input_file = test_dir_ + "/test.ply";
    createTestPointCloud(input_file, 500);
    
    std::vector<int> block_sizes = {4, 8, 13, 16};
    
    for (int block_size : block_sizes) {
        std::cout << "\n=== Testing block size " << block_size << " ===" << std::endl;
        
        std::string compressed_prefix = test_dir_ + "/compressed_" + std::to_string(block_size);
        std::string decompressed_file = test_dir_ + "/decompressed_" + std::to_string(block_size) + ".ply";
        
        CompressionSettings settings;
        settings.voxel_size = 0.5f;
        settings.block_size = block_size;
        settings.min_points_threshold = 1;
        
        PointCloudCompressor compressor(settings);
        
        // Compress
        auto result = compressor.compress(input_file, compressed_prefix);
        ASSERT_TRUE(result.success) << "Compression failed for block size " << block_size;
        
        // Get original grid
        auto original_grid = compressor.getCachedVoxelGrid();
        ASSERT_TRUE(original_grid.has_value());
        
        // Decompress
        PointCloudCompressor decompressor(settings);
        ASSERT_TRUE(decompressor.decompress(compressed_prefix, decompressed_file));
        
        // Load and voxelize decompressed cloud
        PointCloud decompressed_cloud;
        ASSERT_TRUE(PlyIO::readPlyFile(decompressed_file, decompressed_cloud));
        
        VoxelProcessor voxel_processor(settings.voxel_size, settings.block_size, 
                                       settings.min_points_threshold);
        VoxelGrid reconstructed_grid;
        ASSERT_TRUE(voxel_processor.voxelizePointCloud(decompressed_cloud, reconstructed_grid));
        
        // Compare
        std::vector<std::string> differences;
        bool grids_match = compareVoxelGrids(original_grid.value(), reconstructed_grid, differences);
        
        if (!grids_match) {
            std::cout << "Block size " << block_size << " - differences found:" << std::endl;
            for (const auto& diff : differences) {
                std::cout << "  - " << diff << std::endl;
            }
        }
        
        EXPECT_TRUE(grids_match) << "Voxel grids do not match for block size " << block_size;
    }
}

// Test with large number of unique patterns (triggers different index sizes)
TEST_F(VoxelRoundTripTest, LargePatternCount) {
    std::string input_file = test_dir_ + "/test_large.ply";
    
    // Create a more complex point cloud that will generate many unique patterns
    PointCloud cloud;
    for (float x = -10; x <= 10; x += 0.3f) {
        for (float y = -10; y <= 10; y += 0.3f) {
            for (float z = -5; z <= 5; z += 0.3f) {
                // Create a pattern with some randomness
                if ((int(x * 10) + int(y * 10) + int(z * 10)) % 7 < 4) {
                    cloud.points.push_back({x, y, z});
                }
            }
        }
    }
    
    PlyIO::writePlyFile(input_file, cloud);
    std::cout << "Created test cloud with " << cloud.points.size() << " points" << std::endl;
    
    std::string compressed_prefix = test_dir_ + "/compressed_large";
    std::string decompressed_file = test_dir_ + "/decompressed_large.ply";
    
    CompressionSettings settings;
    settings.voxel_size = 0.2f;
    settings.block_size = 8;
    settings.min_points_threshold = 1;
    
    PointCloudCompressor compressor(settings);
    
    // Compress
    auto result = compressor.compress(input_file, compressed_prefix);
    ASSERT_TRUE(result.success) << "Compression failed: " << result.error_message;
    
    std::cout << "Large pattern test stats:" << std::endl;
    std::cout << "  Unique patterns: " << result.num_unique_patterns << std::endl;
    std::cout << "  Index bit size: " << result.index_bit_size << " bits" << std::endl;
    std::cout << "  Max index: " << result.max_index << std::endl;
    
    // Get original grid
    auto original_grid = compressor.getCachedVoxelGrid();
    ASSERT_TRUE(original_grid.has_value());
    
    // Decompress
    PointCloudCompressor decompressor(settings);
    ASSERT_TRUE(decompressor.decompress(compressed_prefix, decompressed_file));
    
    // Load and voxelize decompressed cloud
    PointCloud decompressed_cloud;
    ASSERT_TRUE(PlyIO::readPlyFile(decompressed_file, decompressed_cloud));
    
    VoxelProcessor voxel_processor(settings.voxel_size, settings.block_size, 
                                   settings.min_points_threshold);
    VoxelGrid reconstructed_grid;
    ASSERT_TRUE(voxel_processor.voxelizePointCloud(decompressed_cloud, reconstructed_grid));
    
    // Compare
    std::vector<std::string> differences;
    bool grids_match = compareVoxelGrids(original_grid.value(), reconstructed_grid, differences);
    
    if (!grids_match) {
        std::cout << "\nLarge pattern test - differences found:" << std::endl;
        for (const auto& diff : differences) {
            std::cout << "  - " << diff << std::endl;
        }
    }
    
    EXPECT_TRUE(grids_match) << "Voxel grids do not match with large pattern count";
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}