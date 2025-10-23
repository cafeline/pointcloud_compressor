// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/io/PcdIO.hpp"

using namespace pointcloud_compressor;

class BlockSizeOptimizationTest : public ::testing::Test {
protected:
    std::string test_dir;
    std::string test_pcd_file;
    std::string medium_test_pcd_file;
    
    void SetUp() override {
        test_dir = std::filesystem::temp_directory_path() / "block_size_test";
        std::filesystem::create_directories(test_dir);
        
        test_pcd_file = test_dir + "/test_small.pcd";
        medium_test_pcd_file = test_dir + "/test_medium.pcd";
        
        createSmallTestPcdFile();
        createMediumTestPcdFile();
    }
    
    void TearDown() override {
        std::filesystem::remove_all(test_dir);
    }
    
    void createSmallTestPcdFile() {
        PointCloud cloud;
        // Create a small 5x5x5 grid of points
        for (int x = 0; x < 5; x++) {
            for (int y = 0; y < 5; y++) {
                for (int z = 0; z < 5; z++) {
                    cloud.points.push_back({
                        static_cast<float>(x) * 0.01f,
                        static_cast<float>(y) * 0.01f, 
                        static_cast<float>(z) * 0.01f
                    });
                }
            }
        }
        PcdIO::writePcdFile(test_pcd_file, cloud);
    }
    
    void createMediumTestPcdFile() {
        PointCloud cloud;
        // Create a medium 20x20x20 grid of points
        for (int x = 0; x < 20; x++) {
            for (int y = 0; y < 20; y++) {
                for (int z = 0; z < 20; z++) {
                    cloud.points.push_back({
                        static_cast<float>(x) * 0.01f,
                        static_cast<float>(y) * 0.01f,
                        static_cast<float>(z) * 0.01f
                    });
                }
            }
        }
        PcdIO::writePcdFile(medium_test_pcd_file, cloud);
    }
};

// Test Case 1: Basic optimization functionality
TEST_F(BlockSizeOptimizationTest, BasicOptimization) {
    CompressionSettings settings(0.01f, 8);
    PointCloudCompressor compressor(settings);
    
    // Test optimization with small range
    auto result = compressor.findOptimalBlockSize(test_pcd_file, 4, 8, 1, false);
    
    // Check that we got a valid result
    EXPECT_GE(result.optimal_block_size, 4);
    EXPECT_LE(result.optimal_block_size, 8);
    EXPECT_GT(result.best_compression_ratio, 0.0f);
    EXPECT_LE(result.best_compression_ratio, 1.0f);
    
    // Check that all sizes in range were tested
    EXPECT_EQ(result.tested_results.size(), 5); // 4,5,6,7,8
    for (int i = 4; i <= 8; i++) {
        EXPECT_TRUE(result.tested_results.find(i) != result.tested_results.end());
    }
}

// Test Case 2: Range validation
TEST_F(BlockSizeOptimizationTest, RangeValidation) {
    CompressionSettings settings(0.01f, 8);
    PointCloudCompressor compressor(settings);
    
    // Test min > max (should handle gracefully)
    auto result1 = compressor.findOptimalBlockSize(test_pcd_file, 10, 5, 1, false);
    EXPECT_EQ(result1.optimal_block_size, -1); // Invalid result
    
    // Test min < 1 (should clamp to 1)
    auto result2 = compressor.findOptimalBlockSize(test_pcd_file, 0, 4, 1, false);
    EXPECT_GE(result2.optimal_block_size, 1);
    
    // Test max > reasonable limit (should work but be clamped)
    auto result3 = compressor.findOptimalBlockSize(test_pcd_file, 4, 100, 20, false);
    EXPECT_LE(result3.optimal_block_size, 64); // Reasonable upper limit
}

// Test Case 3: Step size handling
TEST_F(BlockSizeOptimizationTest, StepSizeHandling) {
    CompressionSettings settings(0.01f, 8);
    PointCloudCompressor compressor(settings);
    
    // Test with step_size = 2 (should test only even sizes)
    auto result1 = compressor.findOptimalBlockSize(test_pcd_file, 4, 10, 2, false);
    EXPECT_EQ(result1.tested_results.size(), 4); // 4,6,8,10
    EXPECT_TRUE(result1.tested_results.find(4) != result1.tested_results.end());
    EXPECT_TRUE(result1.tested_results.find(6) != result1.tested_results.end());
    EXPECT_TRUE(result1.tested_results.find(8) != result1.tested_results.end());
    EXPECT_TRUE(result1.tested_results.find(10) != result1.tested_results.end());
    EXPECT_TRUE(result1.tested_results.find(5) == result1.tested_results.end());
    
    // Test with step_size = 3
    auto result2 = compressor.findOptimalBlockSize(test_pcd_file, 3, 12, 3, false);
    EXPECT_EQ(result2.tested_results.size(), 4); // 3,6,9,12
    EXPECT_TRUE(result2.tested_results.find(3) != result2.tested_results.end());
    EXPECT_TRUE(result2.tested_results.find(6) != result2.tested_results.end());
    EXPECT_TRUE(result2.tested_results.find(9) != result2.tested_results.end());
    EXPECT_TRUE(result2.tested_results.find(12) != result2.tested_results.end());
}

// Test Case 4: Compression ratio accuracy
TEST_F(BlockSizeOptimizationTest, CompressionRatioAccuracy) {
    CompressionSettings settings(0.01f, 8);
    PointCloudCompressor compressor(settings);
    
    auto result = compressor.findOptimalBlockSize(medium_test_pcd_file, 4, 16, 4, false);
    
    // Verify each tested block size has a valid compression ratio
    for (const auto& [block_size, ratio] : result.tested_results) {
        EXPECT_GT(ratio, 0.0f);
        EXPECT_LE(ratio, 1.0f);
    }
    
    // The best compression ratio should match the optimal block size
    EXPECT_FLOAT_EQ(result.best_compression_ratio, 
                    result.tested_results[result.optimal_block_size]);
}

// Test Case 5: Detailed results
TEST_F(BlockSizeOptimizationTest, DetailedResults) {
    CompressionSettings settings(0.01f, 8);
    PointCloudCompressor compressor(settings);
    
    auto result = compressor.findOptimalBlockSize(test_pcd_file, 5, 7, 1, true); // verbose=true
    
    // Check that optimization time is measured
    EXPECT_GT(result.optimization_time_ms, 0.0);
    
    // Check that all tested results are recorded
    EXPECT_EQ(result.tested_results.size(), 3); // 5,6,7
    
    // Check that optimal block size is one of the tested sizes
    EXPECT_TRUE(result.tested_results.find(result.optimal_block_size) != 
                result.tested_results.end());
}

// Test Case 6: Integration with compression
TEST_F(BlockSizeOptimizationTest, IntegrationWithCompression) {
    CompressionSettings settings(0.01f, 8);
    PointCloudCompressor compressor(settings);
    
    // Find optimal block size
    auto opt_result = compressor.findOptimalBlockSize(medium_test_pcd_file, 4, 12, 2, false);
    
    // Use the optimal block size for actual compression
    settings.block_size = opt_result.optimal_block_size;
    compressor.updateSettings(settings);
    
    std::string output_prefix = test_dir + "/optimized_compression";
    auto comp_result = compressor.compress(medium_test_pcd_file, output_prefix);
    
    EXPECT_TRUE(comp_result.success);
    // The actual compression ratio should be close to the predicted one
    EXPECT_NEAR(comp_result.compression_ratio, opt_result.best_compression_ratio, 0.05f);
}

// Test Case 7: Performance with larger data
TEST_F(BlockSizeOptimizationTest, LargeDataPerformance) {
    // Create a larger test file
    std::string large_file = test_dir + "/test_large.pcd";
    PointCloud cloud;
    for (int i = 0; i < 10000; i++) {
        cloud.points.push_back({
            static_cast<float>(rand() % 100) * 0.01f,
            static_cast<float>(rand() % 100) * 0.01f,
            static_cast<float>(rand() % 100) * 0.01f
        });
    }
    PcdIO::writePcdFile(large_file, cloud);
    
    CompressionSettings settings(0.01f, 8);
    PointCloudCompressor compressor(settings);
    
    // Test with reasonable range and step to avoid timeout
    auto start_time = std::chrono::high_resolution_clock::now();
    auto result = compressor.findOptimalBlockSize(large_file, 4, 16, 4, false);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    
    // Should complete within reasonable time (30 seconds)
    EXPECT_LT(duration.count(), 30);
    EXPECT_GT(result.optimal_block_size, 0);
}

// Test Case 8: Edge cases
TEST_F(BlockSizeOptimizationTest, EdgeCases) {
    CompressionSettings settings(0.01f, 8);
    PointCloudCompressor compressor(settings);
    
    // Test with empty point cloud
    std::string empty_file = test_dir + "/empty.pcd";
    PointCloud empty_cloud;
    PcdIO::writePcdFile(empty_file, empty_cloud);
    
    auto result1 = compressor.findOptimalBlockSize(empty_file, 4, 8, 1, false);
    EXPECT_EQ(result1.optimal_block_size, -1); // Should indicate failure
    
    // Test with single point
    std::string single_file = test_dir + "/single.pcd";
    PointCloud single_cloud;
    single_cloud.points.push_back({0.0f, 0.0f, 0.0f});
    PcdIO::writePcdFile(single_file, single_cloud);
    
    auto result2 = compressor.findOptimalBlockSize(single_file, 4, 8, 1, false);
    // Should handle gracefully, even if compression isn't meaningful
    EXPECT_GE(result2.optimal_block_size, 4);
    
    // Test with block_size = 1
    auto result3 = compressor.findOptimalBlockSize(test_pcd_file, 1, 1, 1, false);
    EXPECT_EQ(result3.optimal_block_size, 1);
    EXPECT_EQ(result3.tested_results.size(), 1);
}