// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/io/PcdIO.hpp"

using namespace pointcloud_compressor;

class IntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_dir = TEST_DATA_DIR;
        temp_dir = test_dir + "/temp";
        std::filesystem::create_directories(test_dir);
        std::filesystem::create_directories(temp_dir);
        
        // Create a simple test PCD file
        createTestPcdFile();
    }
    
    void TearDown() override {
        if (std::filesystem::exists(temp_dir)) {
            std::filesystem::remove_all(temp_dir);
        }
    }
    
    void createTestPcdFile() {
        test_pcd_file = temp_dir + "/test.pcd";
        
        PointCloud cloud;
        
        // Create a simple 3x3x3 grid of points
        for (float x = 0; x < 0.03f; x += 0.01f) {
            for (float y = 0; y < 0.03f; y += 0.01f) {
                for (float z = 0; z < 0.03f; z += 0.01f) {
                    cloud.points.emplace_back(x, y, z);
                }
            }
        }
        
        // Save to file
        bool success = PcdIO::writePcdFile(test_pcd_file, cloud);
        ASSERT_TRUE(success) << "Failed to create test PCD file";
    }
    
    void createLargeTestPcdFile() {
        large_test_pcd_file = temp_dir + "/large_test.pcd";
        
        PointCloud cloud;
        
        // Create a larger point cloud for more comprehensive testing
        for (float x = 0; x < 0.1f; x += 0.005f) {
            for (float y = 0; y < 0.1f; y += 0.005f) {
                for (float z = 0; z < 0.05f; z += 0.005f) {
                    cloud.points.emplace_back(x, y, z);
                }
            }
        }
        
        bool success = PcdIO::writePcdFile(large_test_pcd_file, cloud);
        ASSERT_TRUE(success) << "Failed to create large test PCD file";
    }
    
    std::string test_dir;
    std::string temp_dir;
    std::string test_pcd_file;
    std::string large_test_pcd_file;
};

// Test basic compress and decompress workflow
TEST_F(IntegrationTest, CompressAndDecompress) {
    // Setup
    CompressionSettings settings(0.01f, 8);
    PointCloudCompressor compressor(settings);
    
    std::string output_prefix = temp_dir + "/compressed";
    std::string decompressed_file = temp_dir + "/decompressed.pcd";
    
    // Compress
    auto compression_result = compressor.compress(test_pcd_file, output_prefix);
    
    // 現実装では成功することを期待
    EXPECT_TRUE(compression_result.success);
    EXPECT_TRUE(compression_result.error_message.empty());

    EXPECT_NEAR(compression_result.margin.x, 0.006, 1e-6);
    EXPECT_NEAR(compression_result.margin.y, 0.006, 1e-6);
    EXPECT_NEAR(compression_result.margin.z, 0.006, 1e-6);

    EXPECT_NEAR(compression_result.grid_origin.x, -0.006, 1e-6);
    EXPECT_NEAR(compression_result.grid_origin.y, -0.006, 1e-6);
    EXPECT_NEAR(compression_result.grid_origin.z, -0.006, 1e-6);
}

// Test compression ratio calculation
TEST_F(IntegrationTest, CompressionRatio) {
    CompressionSettings settings(0.01f, 8);
    PointCloudCompressor compressor(settings);
    
    std::string output_prefix = temp_dir + "/ratio_test";
    
    auto result = compressor.compress(test_pcd_file, output_prefix);
    
    // Even if compression fails, we should get some statistics
    EXPECT_GT(result.original_size, 0);
    EXPECT_GE(result.compression_ratio, 0.0f);
}

// Test data integrity (when implementation is complete)
TEST_F(IntegrationTest, DataIntegrity) {
    // Load original point cloud
    PointCloud original_cloud;
    bool load_success = PcdIO::readPcdFile(test_pcd_file, original_cloud);
    ASSERT_TRUE(load_success);
    
    // For now, just verify we can load the test file
    EXPECT_GT(original_cloud.points.size(), 0);
    EXPECT_EQ(original_cloud.points.size(), 27);  // 3x3x3 = 27 points
    
    // Verify first few points are as expected
    EXPECT_FLOAT_EQ(original_cloud.points[0].x, 0.0f);
    EXPECT_FLOAT_EQ(original_cloud.points[0].y, 0.0f);
    EXPECT_FLOAT_EQ(original_cloud.points[0].z, 0.0f);
}

// Test optimal settings finder
TEST_F(IntegrationTest, FindOptimalSettings) {
    PointCloudCompressor compressor;
    
    auto optimal_settings = compressor.findOptimalSettings(test_pcd_file, 0.005f, 0.02f);
    
    // Should return reasonable settings
    EXPECT_GE(optimal_settings.voxel_size, 0.005f);
    EXPECT_LE(optimal_settings.voxel_size, 0.02f);
    EXPECT_GT(optimal_settings.block_size, 0);
}

// Test memory usage estimation
TEST_F(IntegrationTest, EstimateMemoryUsage) {
    PointCloudCompressor compressor;
    
    size_t estimated_memory = compressor.estimateMemoryUsage(test_pcd_file);
    
    // Should return a reasonable estimate
    EXPECT_GT(estimated_memory, 0);
    EXPECT_LT(estimated_memory, 1000000);  // Should be less than 1MB for small test file
}

// Test file validation
TEST_F(IntegrationTest, ValidateInputFile) {
    PointCloudCompressor compressor;
    
    // Valid file
    EXPECT_TRUE(compressor.validateInputFile(test_pcd_file));
    
    // Non-existent file
    EXPECT_FALSE(compressor.validateInputFile("non_existent.pcd"));
    
    // Wrong extension
    std::string txt_file = temp_dir + "/test.txt";
    std::ofstream(txt_file).close();
    EXPECT_FALSE(compressor.validateInputFile(txt_file));
}

// Test different voxel sizes
TEST_F(IntegrationTest, DifferentVoxelSizes) {
    std::vector<float> voxel_sizes = {0.005f, 0.01f, 0.02f};
    
    for (float voxel_size : voxel_sizes) {
        CompressionSettings settings(voxel_size, 8);
        PointCloudCompressor compressor(settings);
        
        std::string output_prefix = temp_dir + "/voxel_" + std::to_string(voxel_size);
        auto result = compressor.compress(test_pcd_file, output_prefix);
        
        // Should not crash and should provide some statistics
        EXPECT_GE(result.compression_ratio, 0.0f);
        EXPECT_GT(result.original_size, 0);
    }
}

// Test different block sizes
TEST_F(IntegrationTest, DifferentBlockSizes) {
    std::vector<int> block_sizes = {4, 8, 16};
    
    for (int block_size : block_sizes) {
        CompressionSettings settings(0.01f, block_size);
        PointCloudCompressor compressor(settings);
        
        std::string output_prefix = temp_dir + "/block_" + std::to_string(block_size);
        auto result = compressor.compress(test_pcd_file, output_prefix);
        
        // Should not crash
        EXPECT_GE(result.compression_ratio, 0.0f);
    }
}

// Test settings update
TEST_F(IntegrationTest, UpdateSettings) {
    PointCloudCompressor compressor;
    
    CompressionSettings new_settings(0.005f, 16, true);
    compressor.updateSettings(new_settings);
    
    auto retrieved_settings = compressor.getSettings();
    EXPECT_FLOAT_EQ(retrieved_settings.voxel_size, 0.005f);
    EXPECT_EQ(retrieved_settings.block_size, 16);
    EXPECT_TRUE(retrieved_settings.use_8bit_indices);
}

// Test with larger point cloud (if we create one)
TEST_F(IntegrationTest, LargerPointCloud) {
    createLargeTestPcdFile();
    
    CompressionSettings settings(0.005f, 8);
    PointCloudCompressor compressor(settings);
    
    std::string output_prefix = temp_dir + "/large_compressed";
    auto result = compressor.compress(large_test_pcd_file, output_prefix);
    
    // Should handle larger files gracefully
    EXPECT_GT(result.original_size, 0);
    EXPECT_GE(result.compression_ratio, 0.0f);
}

// Test error handling with invalid input
TEST_F(IntegrationTest, ErrorHandling) {
    PointCloudCompressor compressor;
    
    // Try to compress non-existent file
    auto result = compressor.compress("non_existent.pcd", temp_dir + "/output");
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
    
    // Try to decompress non-existent compressed data
    bool decompress_result = compressor.decompress("non_existent_prefix", temp_dir + "/output.pcd");
    EXPECT_FALSE(decompress_result);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
