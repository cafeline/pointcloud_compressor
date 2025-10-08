#include <gtest/gtest.h>
#include "pointcloud_compressor/io/PointCloudIO.hpp"
#include <filesystem>

using namespace pointcloud_compressor;

class PointCloudIOTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_data_dir = TEST_DATA_DIR;
        temp_dir = "/tmp";
        
        // Create test point cloud
        test_cloud.points.clear();
        test_cloud.points.push_back(Point3D(0.0f, 0.0f, 0.0f));
        test_cloud.points.push_back(Point3D(1.0f, 0.0f, 0.0f));
        test_cloud.points.push_back(Point3D(0.0f, 1.0f, 0.0f));
        test_cloud.points.push_back(Point3D(0.0f, 0.0f, 1.0f));
    }
    
    void TearDown() override {
        // Clean up test files
        std::vector<std::string> test_files = {
            temp_dir + "/test_output.pcd",
            temp_dir + "/test_output.ply",
            temp_dir + "/round_trip.pcd",
            temp_dir + "/round_trip.ply"
        };
        
        for (const auto& file : test_files) {
            std::filesystem::remove(file);
        }
    }
    
    std::string test_data_dir;
    std::string temp_dir;
    PointCloud test_cloud;
};

// Test basic auto-detection loading
TEST_F(PointCloudIOTest, LoadPointCloudAutoDetect) {
    // Test PLY file loading
    PointCloud cloud;
    std::string ply_file = test_data_dir + "/cube.ply";
    
    ASSERT_TRUE(PointCloudIO::loadPointCloud(ply_file, cloud));
    EXPECT_EQ(cloud.size(), 8);
    
    // Test another PLY file
    cloud.clear();
    std::string triangle_file = test_data_dir + "/triangle.ply";
    ASSERT_TRUE(PointCloudIO::loadPointCloud(triangle_file, cloud));
    EXPECT_EQ(cloud.size(), 3);
    
    // Test PCD file if exists
    std::string pcd_file = test_data_dir + "/sample.pcd";
    if (std::filesystem::exists(pcd_file)) {
        cloud.clear();
        ASSERT_TRUE(PointCloudIO::loadPointCloud(pcd_file, cloud));
        EXPECT_GT(cloud.size(), 0);
    }
}

// Test explicit format loading
TEST_F(PointCloudIOTest, LoadPointCloudExplicitFormat) {
    PointCloud cloud;
    std::string ply_file = test_data_dir + "/cube.ply";
    
    // Load PLY with explicit format
    ASSERT_TRUE(PointCloudIO::loadPointCloud(ply_file, cloud, FileFormat::PLY));
    EXPECT_EQ(cloud.size(), 8);
    
    // Try to load PLY as PCD (should fail)
    cloud.clear();
    EXPECT_FALSE(PointCloudIO::loadPointCloud(ply_file, cloud, FileFormat::PCD));
    EXPECT_EQ(cloud.size(), 0);
}

// Test saving point clouds
TEST_F(PointCloudIOTest, SavePointCloud) {
    std::string pcd_file = temp_dir + "/test_output.pcd";
    std::string ply_file = temp_dir + "/test_output.ply";
    
    // Save as PCD
    ASSERT_TRUE(PointCloudIO::savePointCloud(pcd_file, test_cloud));
    EXPECT_TRUE(std::filesystem::exists(pcd_file));
    
    // Save as PLY
    ASSERT_TRUE(PointCloudIO::savePointCloud(ply_file, test_cloud));
    EXPECT_TRUE(std::filesystem::exists(ply_file));
    
    // Verify saved files can be loaded back
    PointCloud loaded_pcd, loaded_ply;
    ASSERT_TRUE(PointCloudIO::loadPointCloud(pcd_file, loaded_pcd));
    ASSERT_TRUE(PointCloudIO::loadPointCloud(ply_file, loaded_ply));
    
    EXPECT_EQ(loaded_pcd.size(), test_cloud.size());
    EXPECT_EQ(loaded_ply.size(), test_cloud.size());
}

// Test saving with explicit format
TEST_F(PointCloudIOTest, SavePointCloudExplicitFormat) {
    std::string pcd_file = temp_dir + "/test_output.pcd";
    std::string ply_file = temp_dir + "/test_output.ply";
    
    // Save with explicit formats
    ASSERT_TRUE(PointCloudIO::savePointCloud(pcd_file, test_cloud, FileFormat::PCD));
    ASSERT_TRUE(PointCloudIO::savePointCloud(ply_file, test_cloud, FileFormat::PLY));
    
    // Try to save with mismatched format (should use the specified format)
    std::string pcd_as_ply = temp_dir + "/pcd_format.ply";
    ASSERT_TRUE(PointCloudIO::savePointCloud(pcd_as_ply, test_cloud, FileFormat::PCD));
    
    // The file should be PCD format despite .ply extension
    PointCloud loaded;
    ASSERT_TRUE(PointCloudIO::loadPointCloud(pcd_as_ply, loaded, FileFormat::PCD));
    EXPECT_EQ(loaded.size(), test_cloud.size());
    
    std::filesystem::remove(pcd_as_ply);
}

// Test detailed file information
TEST_F(PointCloudIOTest, LoadPointCloudWithInfo) {
    std::string ply_file = test_data_dir + "/cube.ply";
    PointCloud cloud;
    PointCloudIO::FileInfo info;
    
    ASSERT_TRUE(PointCloudIO::loadPointCloudWithInfo(ply_file, cloud, info));
    
    EXPECT_EQ(info.detected_format, FileFormat::PLY);
    EXPECT_EQ(info.extension_format, FileFormat::PLY);
    EXPECT_EQ(info.point_count, 8);
    EXPECT_FALSE(info.format_mismatch);
    EXPECT_TRUE(info.error_message.empty());
}

// Test format mismatch detection
TEST_F(PointCloudIOTest, FormatMismatchDetection) {
    // Create a test file with PCD content but PLY extension
    std::string mixed_file = temp_dir + "/pcd_content.ply";
    std::ofstream file(mixed_file);
    file << "# .PCD v0.7 - Point Cloud Data file format\n";
    file << "VERSION 0.7\n";
    file << "FIELDS x y z\n";
    file << "SIZE 4 4 4\n";
    file << "TYPE F F F\n";
    file << "COUNT 1 1 1\n";
    file << "WIDTH 2\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS 2\n";
    file << "DATA ascii\n";
    file << "0.0 0.0 0.0\n";
    file << "1.0 1.0 1.0\n";
    file.close();
    
    PointCloud cloud;
    PointCloudIO::FileInfo info;
    
    ASSERT_TRUE(PointCloudIO::loadPointCloudWithInfo(mixed_file, cloud, info));
    
    EXPECT_EQ(info.detected_format, FileFormat::PCD);
    EXPECT_EQ(info.extension_format, FileFormat::PLY);
    EXPECT_TRUE(info.format_mismatch);
    EXPECT_EQ(cloud.size(), 2);
    
    std::filesystem::remove(mixed_file);
}

// Test file validation
TEST_F(PointCloudIOTest, ValidateFile) {
    std::string valid_ply = test_data_dir + "/cube.ply";
    std::string invalid_file = test_data_dir + "/invalid.ply";
    std::string nonexistent = "/nonexistent/file.pcd";
    
    EXPECT_TRUE(PointCloudIO::validateFile(valid_ply));
    EXPECT_FALSE(PointCloudIO::validateFile(invalid_file));
    EXPECT_FALSE(PointCloudIO::validateFile(nonexistent));
}

// Test utility functions
TEST_F(PointCloudIOTest, UtilityFunctions) {
    // Test supported extensions
    auto extensions = PointCloudIO::getSupportedExtensions();
    EXPECT_GE(extensions.size(), 2);
    EXPECT_TRUE(std::find(extensions.begin(), extensions.end(), ".pcd") != extensions.end());
    EXPECT_TRUE(std::find(extensions.begin(), extensions.end(), ".ply") != extensions.end());
    
    // Test supported extension checking
    EXPECT_TRUE(PointCloudIO::hasSupportedExtension("file.pcd"));
    EXPECT_TRUE(PointCloudIO::hasSupportedExtension("file.PLY"));
    EXPECT_TRUE(PointCloudIO::hasSupportedExtension("path/to/file.ply"));
    EXPECT_FALSE(PointCloudIO::hasSupportedExtension("file.txt"));
    EXPECT_FALSE(PointCloudIO::hasSupportedExtension("file"));
}

// Test error handling
TEST_F(PointCloudIOTest, ErrorHandling) {
    PointCloud cloud;
    
    // Test loading non-existent file
    EXPECT_FALSE(PointCloudIO::loadPointCloud("/nonexistent/file.pcd", cloud));
    EXPECT_TRUE(cloud.empty());
    
    // Test loading directory
    EXPECT_FALSE(PointCloudIO::loadPointCloud("/tmp", cloud));
    
    // Test saving to invalid path
    EXPECT_FALSE(PointCloudIO::savePointCloud("/nonexistent/dir/file.pcd", test_cloud));
    
    // Test unsupported format
    EXPECT_FALSE(PointCloudIO::loadPointCloud("file.txt", cloud, FileFormat::UNKNOWN));
    EXPECT_FALSE(PointCloudIO::savePointCloud("file.txt", test_cloud, FileFormat::UNKNOWN));
}

// Test round-trip consistency
TEST_F(PointCloudIOTest, RoundTripConsistency) {
    std::string pcd_file = temp_dir + "/round_trip.pcd";
    std::string ply_file = temp_dir + "/round_trip.ply";
    
    // Save original cloud
    ASSERT_TRUE(PointCloudIO::savePointCloud(pcd_file, test_cloud));
    ASSERT_TRUE(PointCloudIO::savePointCloud(ply_file, test_cloud));
    
    // Load back
    PointCloud loaded_pcd, loaded_ply;
    ASSERT_TRUE(PointCloudIO::loadPointCloud(pcd_file, loaded_pcd));
    ASSERT_TRUE(PointCloudIO::loadPointCloud(ply_file, loaded_ply));
    
    // Compare
    ASSERT_EQ(test_cloud.size(), loaded_pcd.size());
    ASSERT_EQ(test_cloud.size(), loaded_ply.size());
    
    for (size_t i = 0; i < test_cloud.size(); ++i) {
        EXPECT_FLOAT_EQ(test_cloud.points[i].x, loaded_pcd.points[i].x);
        EXPECT_FLOAT_EQ(test_cloud.points[i].y, loaded_pcd.points[i].y);
        EXPECT_FLOAT_EQ(test_cloud.points[i].z, loaded_pcd.points[i].z);
        
        EXPECT_FLOAT_EQ(test_cloud.points[i].x, loaded_ply.points[i].x);
        EXPECT_FLOAT_EQ(test_cloud.points[i].y, loaded_ply.points[i].y);
        EXPECT_FLOAT_EQ(test_cloud.points[i].z, loaded_ply.points[i].z);
    }
}

// Test empty file handling
TEST_F(PointCloudIOTest, EmptyFileHandling) {
    std::string empty_ply = test_data_dir + "/empty.ply";
    PointCloud cloud;
    
    ASSERT_TRUE(PointCloudIO::loadPointCloud(empty_ply, cloud));
    EXPECT_TRUE(cloud.empty());
    
    // Test saving empty cloud
    std::string empty_output = temp_dir + "/empty_output.ply";
    PointCloud empty_cloud;
    ASSERT_TRUE(PointCloudIO::savePointCloud(empty_output, empty_cloud));
    
    // Load back and verify
    PointCloud loaded_empty;
    ASSERT_TRUE(PointCloudIO::loadPointCloud(empty_output, loaded_empty));
    EXPECT_TRUE(loaded_empty.empty());
    
    std::filesystem::remove(empty_output);
}