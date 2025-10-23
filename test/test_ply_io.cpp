// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include "pointcloud_compressor/io/PlyIO.hpp"
#include <filesystem>

using namespace pointcloud_compressor;

class PlyIOTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_data_dir = TEST_DATA_DIR;
    }
    
    std::string test_data_dir;
};

// Test basic PLY file reading
TEST_F(PlyIOTest, ReadBasicPlyFile) {
    PointCloud cloud;
    std::string cube_file = test_data_dir + "/cube.ply";
    
    ASSERT_TRUE(PlyIO::readPlyFile(cube_file, cloud));
    EXPECT_EQ(cloud.size(), 8);
    
    // Check first vertex (0,0,0)
    EXPECT_FLOAT_EQ(cloud.points[0].x, 0.0f);
    EXPECT_FLOAT_EQ(cloud.points[0].y, 0.0f);
    EXPECT_FLOAT_EQ(cloud.points[0].z, 0.0f);
    
    // Check last vertex (0,1,1) 
    EXPECT_FLOAT_EQ(cloud.points[7].x, 0.0f);
    EXPECT_FLOAT_EQ(cloud.points[7].y, 1.0f);
    EXPECT_FLOAT_EQ(cloud.points[7].z, 1.0f);
}

// Test triangle PLY file reading
TEST_F(PlyIOTest, ReadTrianglePlyFile) {
    PointCloud cloud;
    std::string triangle_file = test_data_dir + "/triangle.ply";
    
    ASSERT_TRUE(PlyIO::readPlyFile(triangle_file, cloud));
    EXPECT_EQ(cloud.size(), 3);
    
    // Check triangle vertices
    EXPECT_FLOAT_EQ(cloud.points[0].x, 0.0f);
    EXPECT_FLOAT_EQ(cloud.points[1].x, 1.0f);
    EXPECT_FLOAT_EQ(cloud.points[2].x, 0.5f);
    EXPECT_FLOAT_EQ(cloud.points[2].y, 1.0f);
}

// Test PLY header parsing
TEST_F(PlyIOTest, ParsePlyHeader) {
    PlyHeader header;
    std::string cube_file = test_data_dir + "/cube.ply";
    
    ASSERT_TRUE(PlyIO::parseHeader(cube_file, header));
    EXPECT_EQ(header.format, "ascii");
    EXPECT_EQ(header.version, "1.0");
    EXPECT_EQ(header.vertex_count, 8);
    EXPECT_GT(header.comments.size(), 0);
    EXPECT_TRUE(header.comments[0].find("Simple cube") != std::string::npos);
}

// Test PLY file with comments
TEST_F(PlyIOTest, ReadCommentedPlyFile) {
    PointCloud cloud;
    PlyHeader header;
    std::string commented_file = test_data_dir + "/commented.ply";
    
    ASSERT_TRUE(PlyIO::parseHeader(commented_file, header));
    EXPECT_EQ(header.comments.size(), 3);
    EXPECT_TRUE(header.comments[0].find("Created for testing") != std::string::npos);
    
    ASSERT_TRUE(PlyIO::readPlyFile(commented_file, cloud));
    EXPECT_EQ(cloud.size(), 4);
}

// Test empty PLY file
TEST_F(PlyIOTest, ReadEmptyPlyFile) {
    PointCloud cloud;
    std::string empty_file = test_data_dir + "/empty.ply";
    
    ASSERT_TRUE(PlyIO::readPlyFile(empty_file, cloud));
    EXPECT_EQ(cloud.size(), 0);
    EXPECT_TRUE(cloud.empty());
}

// Test invalid PLY file
TEST_F(PlyIOTest, ReadInvalidPlyFile) {
    PointCloud cloud;
    std::string invalid_file = test_data_dir + "/invalid.ply";
    
    // Should fail gracefully for malformed data
    EXPECT_FALSE(PlyIO::readPlyFile(invalid_file, cloud));
}

// Test non-existent file
TEST_F(PlyIOTest, ReadNonExistentFile) {
    PointCloud cloud;
    std::string nonexistent_file = test_data_dir + "/nonexistent.ply";
    
    EXPECT_FALSE(PlyIO::readPlyFile(nonexistent_file, cloud));
    EXPECT_TRUE(cloud.empty());
}

// Test PLY format validation
TEST_F(PlyIOTest, ValidatePlyFormat) {
    std::string cube_file = test_data_dir + "/cube.ply";
    std::string invalid_file = test_data_dir + "/invalid.ply";
    std::string pcd_file = test_data_dir + "/sample.pcd";
    
    EXPECT_TRUE(PlyIO::validateFormat(cube_file));
    EXPECT_FALSE(PlyIO::validateFormat(invalid_file));
    
    // PCD file should not validate as PLY
    if (std::filesystem::exists(pcd_file)) {
        EXPECT_FALSE(PlyIO::validateFormat(pcd_file));
    }
}

// Test writing PLY file
TEST_F(PlyIOTest, WritePlyFile) {
    // Create test point cloud
    PointCloud cloud;
    cloud.points.push_back(Point3D(0.0f, 0.0f, 0.0f));
    cloud.points.push_back(Point3D(1.0f, 0.0f, 0.0f));
    cloud.points.push_back(Point3D(0.0f, 1.0f, 0.0f));
    
    std::string output_file = "/tmp/test_output.ply";
    
    // Write and read back
    ASSERT_TRUE(PlyIO::writePlyFile(output_file, cloud));
    
    PointCloud read_cloud;
    ASSERT_TRUE(PlyIO::readPlyFile(output_file, read_cloud));
    
    EXPECT_EQ(read_cloud.size(), 3);
    EXPECT_FLOAT_EQ(read_cloud.points[0].x, 0.0f);
    EXPECT_FLOAT_EQ(read_cloud.points[1].x, 1.0f);
    EXPECT_FLOAT_EQ(read_cloud.points[2].y, 1.0f);
    
    // Clean up
    std::filesystem::remove(output_file);
}

// Test round-trip consistency
TEST_F(PlyIOTest, RoundTripConsistency) {
    PointCloud original_cloud;
    std::string cube_file = test_data_dir + "/cube.ply";
    
    // Read original
    ASSERT_TRUE(PlyIO::readPlyFile(cube_file, original_cloud));
    
    // Write to temp file
    std::string temp_file = "/tmp/roundtrip.ply";
    ASSERT_TRUE(PlyIO::writePlyFile(temp_file, original_cloud));
    
    // Read back
    PointCloud round_trip_cloud;
    ASSERT_TRUE(PlyIO::readPlyFile(temp_file, round_trip_cloud));
    
    // Compare
    ASSERT_EQ(original_cloud.size(), round_trip_cloud.size());
    for (size_t i = 0; i < original_cloud.size(); ++i) {
        EXPECT_FLOAT_EQ(original_cloud.points[i].x, round_trip_cloud.points[i].x);
        EXPECT_FLOAT_EQ(original_cloud.points[i].y, round_trip_cloud.points[i].y);  
        EXPECT_FLOAT_EQ(original_cloud.points[i].z, round_trip_cloud.points[i].z);
    }
    
    // Clean up
    std::filesystem::remove(temp_file);
}

// Binary PLY format tests
TEST_F(PlyIOTest, ReadBinaryPlyFile) {
    PointCloud cloud;
    std::string binary_file = "/tmp/test_binary_cube.ply";
    
    ASSERT_TRUE(PlyIO::readPlyFile(binary_file, cloud));
    EXPECT_EQ(cloud.size(), 8);
    
    // Check first vertex (0,0,0)
    EXPECT_FLOAT_EQ(cloud.points[0].x, 0.0f);
    EXPECT_FLOAT_EQ(cloud.points[0].y, 0.0f);
    EXPECT_FLOAT_EQ(cloud.points[0].z, 0.0f);
    
    // Check last vertex (1,1,1)
    EXPECT_FLOAT_EQ(cloud.points[7].x, 1.0f);
    EXPECT_FLOAT_EQ(cloud.points[7].y, 1.0f);
    EXPECT_FLOAT_EQ(cloud.points[7].z, 1.0f);
}

TEST_F(PlyIOTest, ParseBinaryPlyHeader) {
    PlyHeader header;
    std::string binary_file = "/tmp/test_binary_cube.ply";
    
    ASSERT_TRUE(PlyIO::parseHeader(binary_file, header));
    EXPECT_EQ(header.format, "binary_little_endian");
    EXPECT_EQ(header.version, "1.0");
    EXPECT_EQ(header.vertex_count, 8);
}

TEST_F(PlyIOTest, ValidateBinaryPlyFormat) {
    std::string binary_file = "/tmp/test_binary_cube.ply";
    
    EXPECT_TRUE(PlyIO::validateFormat(binary_file));
}

TEST_F(PlyIOTest, ReadLargeBinaryPlyFile) {
    // Create a larger binary PLY file for testing
    std::string large_binary_file = "/tmp/test_large_binary.ply";
    
    // Create test file with 1000 points
    std::ofstream file(large_binary_file, std::ios::binary);
    file << "ply\n";
    file << "format binary_little_endian 1.0\n";
    file << "element vertex 1000\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";
    
    // Write 1000 vertices
    for (int i = 0; i < 1000; ++i) {
        float x = static_cast<float>(i % 10);
        float y = static_cast<float>((i / 10) % 10);
        float z = static_cast<float>(i / 100);
        file.write(reinterpret_cast<const char*>(&x), sizeof(float));
        file.write(reinterpret_cast<const char*>(&y), sizeof(float));
        file.write(reinterpret_cast<const char*>(&z), sizeof(float));
    }
    file.close();
    
    // Test reading
    PointCloud cloud;
    ASSERT_TRUE(PlyIO::readPlyFile(large_binary_file, cloud));
    EXPECT_EQ(cloud.size(), 1000);
    
    // Verify some points
    EXPECT_FLOAT_EQ(cloud.points[0].x, 0.0f);
    EXPECT_FLOAT_EQ(cloud.points[999].x, 9.0f);
    EXPECT_FLOAT_EQ(cloud.points[999].y, 9.0f);
    EXPECT_FLOAT_EQ(cloud.points[999].z, 9.0f);
    
    // Clean up
    std::filesystem::remove(large_binary_file);
}

TEST_F(PlyIOTest, ReadBinaryBigEndianNotSupported) {
    // Create a binary big endian PLY file (should fail)
    std::string big_endian_file = "/tmp/test_big_endian.ply";
    std::ofstream file(big_endian_file);
    file << "ply\n";
    file << "format binary_big_endian 1.0\n";
    file << "element vertex 1\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";
    file.close();
    
    PointCloud cloud;
    // Should fail - big endian not supported yet
    EXPECT_FALSE(PlyIO::readPlyFile(big_endian_file, cloud));
    
    // Clean up
    std::filesystem::remove(big_endian_file);
}