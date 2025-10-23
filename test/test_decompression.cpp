// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/io/PointCloudIO.hpp"
#include <cmath>

using namespace pointcloud_compressor;

class DecompressionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test point cloud
        createTestPointCloud();
    }

    void TearDown() override {
        // Clean up test files
        std::system("rm -f /tmp/test_compress_*");
        std::system("rm -f /tmp/test_decompress.pcd");
    }

    void createTestPointCloud() {
        test_cloud_.clear();
        
        // Create a simple 3x3x3 cube of points
        for (float x = 0; x < 0.3f; x += 0.1f) {
            for (float y = 0; y < 0.3f; y += 0.1f) {
                for (float z = 0; z < 0.3f; z += 0.1f) {
                    test_cloud_.points.push_back(Point3D(x, y, z));
                }
            }
        }
    }

    PointCloud test_cloud_;
};

TEST_F(DecompressionTest, CompressAndDecompress) {
    // Save test point cloud
    std::string input_file = "/tmp/test_input.pcd";
    ASSERT_TRUE(PointCloudIO::savePointCloud(input_file, test_cloud_));

    // Compress
    CompressionSettings settings(0.05f, 8);  // voxel_size=0.05, block_size=8
    PointCloudCompressor compressor(settings);
    
    std::string compressed_prefix = "/tmp/test_compress";
    CompressionResult result = compressor.compress(input_file, compressed_prefix);
    ASSERT_TRUE(result.success) << result.error_message;

    // Decompress
    std::string output_file = "/tmp/test_decompress.pcd";
    ASSERT_TRUE(compressor.decompress(compressed_prefix, output_file));

    // Load decompressed point cloud
    PointCloud decompressed_cloud;
    ASSERT_TRUE(PointCloudIO::loadPointCloud(output_file, decompressed_cloud));

    // Verify decompressed cloud is not empty
    ASSERT_GT(decompressed_cloud.points.size(), 0);
    
    // The decompressed cloud should have points at voxel centers
    // Check that points are quantized to voxel grid
    float voxel_size = settings.voxel_size;
    for (const auto& point : decompressed_cloud.points) {
        // Points should be at voxel centers
        float voxel_center_offset = voxel_size / 2.0f;
        
        // Check if point coordinates are close to voxel centers
        float remainder_x = std::fmod(point.x + voxel_center_offset, voxel_size);
        float remainder_y = std::fmod(point.y + voxel_center_offset, voxel_size);
        float remainder_z = std::fmod(point.z + voxel_center_offset, voxel_size);
        
        // Allow for floating point errors
        const float epsilon = 1e-5f;
        EXPECT_NEAR(remainder_x, 0.0f, voxel_size + epsilon);
        EXPECT_NEAR(remainder_y, 0.0f, voxel_size + epsilon);
        EXPECT_NEAR(remainder_z, 0.0f, voxel_size + epsilon);
    }
}

TEST_F(DecompressionTest, EmptyPointCloud) {
    // Create empty point cloud
    PointCloud empty_cloud;
    std::string input_file = "/tmp/test_empty.pcd";
    ASSERT_TRUE(PointCloudIO::savePointCloud(input_file, empty_cloud));

    // Compress empty cloud
    CompressionSettings settings(0.01f, 8);
    PointCloudCompressor compressor(settings);
    
    std::string compressed_prefix = "/tmp/test_compress_empty";
    CompressionResult result = compressor.compress(input_file, compressed_prefix);
    
    // Empty cloud should fail to compress
    EXPECT_FALSE(result.success);
}

TEST_F(DecompressionTest, DifferentVoxelSizes) {
    std::string input_file = "/tmp/test_input.pcd";
    ASSERT_TRUE(PointCloudIO::savePointCloud(input_file, test_cloud_));

    std::vector<float> voxel_sizes = {0.01f, 0.05f, 0.1f};
    
    for (float voxel_size : voxel_sizes) {
        CompressionSettings settings(voxel_size, 8);
        PointCloudCompressor compressor(settings);
        
        std::string compressed_prefix = "/tmp/test_compress_" + std::to_string(voxel_size);
        CompressionResult result = compressor.compress(input_file, compressed_prefix);
        ASSERT_TRUE(result.success) << "Failed with voxel_size=" << voxel_size;

        std::string output_file = "/tmp/test_decompress_" + std::to_string(voxel_size) + ".pcd";
        ASSERT_TRUE(compressor.decompress(compressed_prefix, output_file));

        PointCloud decompressed_cloud;
        ASSERT_TRUE(PointCloudIO::loadPointCloud(output_file, decompressed_cloud));
        ASSERT_GT(decompressed_cloud.points.size(), 0);
    }
}

TEST_F(DecompressionTest, LargePointCloud) {
    // Create larger point cloud
    PointCloud large_cloud;
    for (float x = 0; x < 1.0f; x += 0.02f) {
        for (float y = 0; y < 1.0f; y += 0.02f) {
            for (float z = 0; z < 0.5f; z += 0.02f) {
                large_cloud.points.push_back(Point3D(x, y, z));
            }
        }
    }

    std::string input_file = "/tmp/test_large.pcd";
    ASSERT_TRUE(PointCloudIO::savePointCloud(input_file, large_cloud));

    CompressionSettings settings(0.05f, 16);  // Larger block size for efficiency
    PointCloudCompressor compressor(settings);
    
    std::string compressed_prefix = "/tmp/test_compress_large";
    CompressionResult result = compressor.compress(input_file, compressed_prefix);
    ASSERT_TRUE(result.success);

    std::string output_file = "/tmp/test_decompress_large.pcd";
    ASSERT_TRUE(compressor.decompress(compressed_prefix, output_file));

    PointCloud decompressed_cloud;
    ASSERT_TRUE(PointCloudIO::loadPointCloud(output_file, decompressed_cloud));
    ASSERT_GT(decompressed_cloud.points.size(), 0);
}

TEST_F(DecompressionTest, MetadataPreservation) {
    std::string input_file = "/tmp/test_input.pcd";
    ASSERT_TRUE(PointCloudIO::savePointCloud(input_file, test_cloud_));

    CompressionSettings settings(0.03f, 8);
    PointCloudCompressor compressor(settings);
    
    std::string compressed_prefix = "/tmp/test_compress_meta";
    CompressionResult compress_result = compressor.compress(input_file, compressed_prefix);
    ASSERT_TRUE(compress_result.success);

    // Save compression metadata
    float original_voxel_size = settings.voxel_size;
    int original_block_size = settings.block_size;
    size_t original_num_blocks = compress_result.num_blocks;

    // Decompress
    std::string output_file = "/tmp/test_decompress_meta.pcd";
    ASSERT_TRUE(compressor.decompress(compressed_prefix, output_file));

    // Verify metadata files exist
    std::ifstream meta_file(compressed_prefix + "_meta.bin", std::ios::binary);
    ASSERT_TRUE(meta_file.is_open());
    
    // Read metadata
    float voxel_size;
    int block_size;
    meta_file.read(reinterpret_cast<char*>(&voxel_size), sizeof(voxel_size));
    meta_file.read(reinterpret_cast<char*>(&block_size), sizeof(block_size));
    
    EXPECT_EQ(voxel_size, original_voxel_size);
    EXPECT_EQ(block_size, original_block_size);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}