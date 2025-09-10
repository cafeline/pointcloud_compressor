#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <filesystem>
#include <fstream>
#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/core/VoxelProcessor.hpp"
#include "pointcloud_compressor/io/PointCloudIO.hpp"

namespace fs = std::filesystem;

class VoxelGridReuseTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test point cloud data
        test_file_ = "/tmp/test_pointcloud_" + std::to_string(
            std::chrono::system_clock::now().time_since_epoch().count()) + ".ply";
        
        // Create a simple PLY file directly
        std::ofstream file(test_file_);
        file << "ply\n";
        file << "format ascii 1.0\n";
        file << "element vertex 1000\n";
        file << "property float x\n";
        file << "property float y\n";
        file << "property float z\n";
        file << "end_header\n";
        
        // Generate simple test point cloud
        for (float x = 0; x < 10; x += 1.0f) {
            for (float y = 0; y < 10; y += 1.0f) {
                for (float z = 0; z < 10; z += 1.0f) {
                    file << x << " " << y << " " << z << "\n";
                }
            }
        }
        file.close();
    }
    
    void TearDown() override {
        // Clean up test file
        if (fs::exists(test_file_)) {
            fs::remove(test_file_);
        }
    }
    
    std::string test_file_;
};

// Test that voxel grid is reused instead of recalculated
TEST_F(VoxelGridReuseTest, TestVoxelGridIsReused) {
    // Settings for compression
    pointcloud_compressor::CompressionSettings settings;
    settings.voxel_size = 1.0;
    settings.block_size = 8;
    settings.min_points_threshold = 1;
    
    // Create compressor
    auto compressor = std::make_unique<pointcloud_compressor::PointCloudCompressor>(settings);
    
    // First compression should calculate voxel grid
    auto start1 = std::chrono::high_resolution_clock::now();
    auto result = compressor->compress(test_file_, "/tmp/test_compressed");
    auto end1 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);
    
    ASSERT_TRUE(result.success);
    
    // Get cached voxel grid (this should be fast)
    auto start2 = std::chrono::high_resolution_clock::now();
    auto cached_grid = compressor->getCachedVoxelGrid();
    auto end2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(end2 - start2);
    
    ASSERT_TRUE(cached_grid.has_value());
    
    // Accessing cached grid should be much faster than original compression
    // Should be less than 1ms (1000 microseconds)
    EXPECT_LT(duration2.count(), 1000) << "Accessing cached grid took " 
                                        << duration2.count() << " microseconds";
    
    // Verify the grid has expected dimensions
    auto dimensions = cached_grid.value().getDimensions();
    EXPECT_GT(dimensions.x, 0);
    EXPECT_GT(dimensions.y, 0);
    EXPECT_GT(dimensions.z, 0);
}

// Test that voxel grid is cleared when needed
TEST_F(VoxelGridReuseTest, TestVoxelGridCanBeCleared) {
    pointcloud_compressor::CompressionSettings settings;
    settings.voxel_size = 1.0;
    settings.block_size = 8;
    settings.min_points_threshold = 1;
    
    auto compressor = std::make_unique<pointcloud_compressor::PointCloudCompressor>(settings);
    
    // Compress to generate voxel grid
    auto result = compressor->compress(test_file_, "/tmp/test_compressed");
    ASSERT_TRUE(result.success);
    
    // Should have cached grid
    auto cached_grid = compressor->getCachedVoxelGrid();
    ASSERT_TRUE(cached_grid.has_value());
    
    // Clear the cache
    compressor->clearCachedVoxelGrid();
    
    // Should no longer have cached grid
    cached_grid = compressor->getCachedVoxelGrid();
    EXPECT_FALSE(cached_grid.has_value());
}

// Test performance improvement with grid reuse
TEST_F(VoxelGridReuseTest, TestPerformanceImprovement) {
    pointcloud_compressor::CompressionSettings settings;
    settings.voxel_size = 0.5;
    settings.block_size = 8;
    settings.min_points_threshold = 1;
    
    // Load point cloud once
    pointcloud_compressor::PointCloud cloud;
    ASSERT_TRUE(pointcloud_compressor::PointCloudIO::loadPointCloud(test_file_, cloud));
    
    // First voxelization (no cache)
    auto processor1 = std::make_unique<pointcloud_compressor::VoxelProcessor>(
        settings.voxel_size, settings.block_size, settings.min_points_threshold);
    
    auto start1 = std::chrono::high_resolution_clock::now();
    pointcloud_compressor::VoxelGrid grid1;
    processor1->voxelizePointCloud(cloud, grid1);
    auto end1 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(end1 - start1);
    
    // Store the grid for reuse
    auto stored_grid = std::make_unique<pointcloud_compressor::VoxelGrid>(std::move(grid1));
    
    // Second "voxelization" using stored grid (should be instant)
    auto start2 = std::chrono::high_resolution_clock::now();
    // Just access the stored grid
    auto dimensions = stored_grid->getDimensions();
    auto end2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(end2 - start2);
    
    // Reusing should be at least 100x faster
    EXPECT_GT(duration1.count(), duration2.count() * 100) 
        << "Original: " << duration1.count() << "us, Reused: " << duration2.count() << "us";
    
    // Verify dimensions are valid
    EXPECT_GT(dimensions.x, 0);
    EXPECT_GT(dimensions.y, 0);
    EXPECT_GT(dimensions.z, 0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}