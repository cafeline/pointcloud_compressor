#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include "pointcloud_compressor/io/PcdIO.hpp"

namespace fs = std::filesystem;

class PcdIOTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test directory if it doesn't exist
        test_dir = TEST_DATA_DIR;
        fs::create_directories(test_dir);
        
        // Create temp directory for output files
        temp_dir = test_dir + "/temp";
        fs::create_directories(temp_dir);
    }
    
    void TearDown() override {
        // Clean up temp files
        if (fs::exists(temp_dir)) {
            fs::remove_all(temp_dir);
        }
    }
    
    void createSimplePcdFile(const std::string& filename) {
        std::ofstream file(filename);
        file << "# .PCD v0.7 - Point Cloud Data file format\n";
        file << "VERSION 0.7\n";
        file << "FIELDS x y z\n";
        file << "SIZE 4 4 4\n";
        file << "TYPE F F F\n";
        file << "COUNT 1 1 1\n";
        file << "WIDTH 3\n";
        file << "HEIGHT 1\n";
        file << "VIEWPOINT 0 0 0 1 0 0 0\n";
        file << "POINTS 3\n";
        file << "DATA ascii\n";
        file << "1.0 2.0 3.0\n";
        file << "4.0 5.0 6.0\n";
        file << "7.0 8.0 9.0\n";
        file.close();
    }
    
    std::string test_dir;
    std::string temp_dir;
};

// Test reading an empty file
TEST_F(PcdIOTest, ReadEmptyFile) {
    std::string empty_file = temp_dir + "/empty.pcd";
    std::ofstream(empty_file).close();  // Create empty file
    
    pointcloud_compressor::PointCloud cloud;
    bool result = pointcloud_compressor::PcdIO::readPcdFile(empty_file, cloud);
    
    EXPECT_FALSE(result);
    EXPECT_EQ(cloud.points.size(), 0);
}

// Test reading a non-existent file
TEST_F(PcdIOTest, ReadNonExistentFile) {
    std::string non_existent = temp_dir + "/non_existent.pcd";
    
    pointcloud_compressor::PointCloud cloud;
    bool result = pointcloud_compressor::PcdIO::readPcdFile(non_existent, cloud);
    
    EXPECT_FALSE(result);
}

// Test reading a valid PCD file
TEST_F(PcdIOTest, ReadValidPcdFile) {
    std::string valid_file = temp_dir + "/valid.pcd";
    createSimplePcdFile(valid_file);
    
    pointcloud_compressor::PointCloud cloud;
    bool result = pointcloud_compressor::PcdIO::readPcdFile(valid_file, cloud);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(cloud.points.size(), 3);
    
    // Check first point
    EXPECT_FLOAT_EQ(cloud.points[0].x, 1.0f);
    EXPECT_FLOAT_EQ(cloud.points[0].y, 2.0f);
    EXPECT_FLOAT_EQ(cloud.points[0].z, 3.0f);
    
    // Check second point
    EXPECT_FLOAT_EQ(cloud.points[1].x, 4.0f);
    EXPECT_FLOAT_EQ(cloud.points[1].y, 5.0f);
    EXPECT_FLOAT_EQ(cloud.points[1].z, 6.0f);
    
    // Check third point
    EXPECT_FLOAT_EQ(cloud.points[2].x, 7.0f);
    EXPECT_FLOAT_EQ(cloud.points[2].y, 8.0f);
    EXPECT_FLOAT_EQ(cloud.points[2].z, 9.0f);
}

// Test writing a valid PCD file
TEST_F(PcdIOTest, WriteValidPcdFile) {
    // Create a point cloud
    pointcloud_compressor::PointCloud cloud;
    cloud.points.push_back({1.0f, 2.0f, 3.0f});
    cloud.points.push_back({4.0f, 5.0f, 6.0f});
    
    std::string output_file = temp_dir + "/output.pcd";
    bool result = pointcloud_compressor::PcdIO::writePcdFile(output_file, cloud);
    
    EXPECT_TRUE(result);
    EXPECT_TRUE(fs::exists(output_file));
    
    // Read back and verify
    pointcloud_compressor::PointCloud read_cloud;
    bool read_result = pointcloud_compressor::PcdIO::readPcdFile(output_file, read_cloud);
    
    EXPECT_TRUE(read_result);
    EXPECT_EQ(read_cloud.points.size(), 2);
    EXPECT_FLOAT_EQ(read_cloud.points[0].x, 1.0f);
    EXPECT_FLOAT_EQ(read_cloud.points[0].y, 2.0f);
    EXPECT_FLOAT_EQ(read_cloud.points[0].z, 3.0f);
}

// Test writing to an invalid path
TEST_F(PcdIOTest, WriteToInvalidPath) {
    pointcloud_compressor::PointCloud cloud;
    cloud.points.push_back({1.0f, 2.0f, 3.0f});
    
    std::string invalid_path = "/invalid/path/output.pcd";
    bool result = pointcloud_compressor::PcdIO::writePcdFile(invalid_path, cloud);
    
    EXPECT_FALSE(result);
}

// Test parsing header correctly
TEST_F(PcdIOTest, ParseHeaderCorrectly) {
    std::string valid_file = temp_dir + "/header_test.pcd";
    createSimplePcdFile(valid_file);
    
    pointcloud_compressor::PcdHeader header;
    bool result = pointcloud_compressor::PcdIO::parseHeader(valid_file, header);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(header.width, 3);
    EXPECT_EQ(header.height, 1);
    EXPECT_EQ(header.points, 3);
    EXPECT_EQ(header.data_type, "ascii");
}

// Test round-trip: write and read back
TEST_F(PcdIOTest, RoundTripWriteRead) {
    // Create original point cloud
    pointcloud_compressor::PointCloud original;
    for (int i = 0; i < 100; ++i) {
        original.points.push_back({
            static_cast<float>(i * 0.1f),
            static_cast<float>(i * 0.2f),
            static_cast<float>(i * 0.3f)
        });
    }
    
    std::string file_path = temp_dir + "/roundtrip.pcd";
    
    // Write
    bool write_result = pointcloud_compressor::PcdIO::writePcdFile(file_path, original);
    ASSERT_TRUE(write_result);
    
    // Read back
    pointcloud_compressor::PointCloud loaded;
    bool read_result = pointcloud_compressor::PcdIO::readPcdFile(file_path, loaded);
    ASSERT_TRUE(read_result);
    
    // Compare
    ASSERT_EQ(original.points.size(), loaded.points.size());
    for (size_t i = 0; i < original.points.size(); ++i) {
        EXPECT_FLOAT_EQ(original.points[i].x, loaded.points[i].x);
        EXPECT_FLOAT_EQ(original.points[i].y, loaded.points[i].y);
        EXPECT_FLOAT_EQ(original.points[i].z, loaded.points[i].z);
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}