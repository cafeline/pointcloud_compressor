#include <gtest/gtest.h>
#include "pointcloud_compressor/io/FileFormatDetector.hpp"
#include <filesystem>
#include <fstream>

using namespace pointcloud_compressor;

class FileFormatDetectorTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_data_dir = TEST_DATA_DIR;
        
        // Create test files with mixed extensions and content
        createMixedFormatFiles();
    }
    
    void TearDown() override {
        // Clean up temporary test files
        std::filesystem::remove("/tmp/pcd_with_ply_ext.ply");
        std::filesystem::remove("/tmp/ply_with_pcd_ext.pcd");
        std::filesystem::remove("/tmp/invalid_file.xyz");
    }
    
    void createMixedFormatFiles() {
        // Create PCD content with PLY extension
        std::ofstream pcd_ply("/tmp/pcd_with_ply_ext.ply");
        pcd_ply << "# .PCD v0.7 - Point Cloud Data file format\n";
        pcd_ply << "VERSION 0.7\n";
        pcd_ply << "FIELDS x y z\n";
        pcd_ply << "SIZE 4 4 4\n";
        pcd_ply << "TYPE F F F\n";
        pcd_ply << "COUNT 1 1 1\n";
        pcd_ply << "WIDTH 3\n";
        pcd_ply << "HEIGHT 1\n";
        pcd_ply << "VIEWPOINT 0 0 0 1 0 0 0\n";
        pcd_ply << "POINTS 3\n";
        pcd_ply << "DATA ascii\n";
        pcd_ply << "0.0 0.0 0.0\n";
        pcd_ply << "1.0 0.0 0.0\n";
        pcd_ply << "0.0 1.0 0.0\n";
        pcd_ply.close();
        
        // Create PLY content with PCD extension
        std::ofstream ply_pcd("/tmp/ply_with_pcd_ext.pcd");
        ply_pcd << "ply\n";
        ply_pcd << "format ascii 1.0\n";
        ply_pcd << "element vertex 3\n";
        ply_pcd << "property float x\n";
        ply_pcd << "property float y\n";
        ply_pcd << "property float z\n";
        ply_pcd << "end_header\n";
        ply_pcd << "0.0 0.0 0.0\n";
        ply_pcd << "1.0 0.0 0.0\n";
        ply_pcd << "0.0 1.0 0.0\n";
        ply_pcd.close();
        
        // Create invalid file with unknown extension
        std::ofstream invalid("/tmp/invalid_file.xyz");
        invalid << "This is not a valid point cloud file\n";
        invalid << "Random content here\n";
        invalid.close();
    }
    
    std::string test_data_dir;
};

// Test format detection by extension
TEST_F(FileFormatDetectorTest, DetectByExtension) {
    // Test standard extensions
    EXPECT_EQ(FileFormatDetector::detectByExtension("file.pcd"), FileFormat::PCD);
    EXPECT_EQ(FileFormatDetector::detectByExtension("file.PCD"), FileFormat::PCD);
    EXPECT_EQ(FileFormatDetector::detectByExtension("file.ply"), FileFormat::PLY);
    EXPECT_EQ(FileFormatDetector::detectByExtension("file.PLY"), FileFormat::PLY);
    
    // Test unknown extensions
    EXPECT_EQ(FileFormatDetector::detectByExtension("file.txt"), FileFormat::UNKNOWN);
    EXPECT_EQ(FileFormatDetector::detectByExtension("file.xyz"), FileFormat::UNKNOWN);
    EXPECT_EQ(FileFormatDetector::detectByExtension("file"), FileFormat::UNKNOWN);
    
    // Test edge cases
    EXPECT_EQ(FileFormatDetector::detectByExtension(""), FileFormat::UNKNOWN);
    EXPECT_EQ(FileFormatDetector::detectByExtension(".pcd"), FileFormat::PCD);
    EXPECT_EQ(FileFormatDetector::detectByExtension("path/to/file.ply"), FileFormat::PLY);
}

// Test format detection by content
TEST_F(FileFormatDetectorTest, DetectByContent) {
    // Test existing test files
    std::string cube_ply = test_data_dir + "/cube.ply";
    EXPECT_EQ(FileFormatDetector::detectByContent(cube_ply), FileFormat::PLY);
    
    std::string triangle_ply = test_data_dir + "/triangle.ply";
    EXPECT_EQ(FileFormatDetector::detectByContent(triangle_ply), FileFormat::PLY);
    
    // Test PCD file if exists
    std::string sample_pcd = test_data_dir + "/sample.pcd";
    if (std::filesystem::exists(sample_pcd)) {
        EXPECT_EQ(FileFormatDetector::detectByContent(sample_pcd), FileFormat::PCD);
    }
    
    // Test invalid files
    EXPECT_EQ(FileFormatDetector::detectByContent("/tmp/invalid_file.xyz"), FileFormat::UNKNOWN);
    EXPECT_EQ(FileFormatDetector::detectByContent("/nonexistent/file.pcd"), FileFormat::UNKNOWN);
}

// Test mixed extension and content scenarios
TEST_F(FileFormatDetectorTest, DetectMixedExtensionContent) {
    // PCD content with PLY extension - content should take priority
    EXPECT_EQ(FileFormatDetector::detectByContent("/tmp/pcd_with_ply_ext.ply"), FileFormat::PCD);
    EXPECT_EQ(FileFormatDetector::detectByExtension("/tmp/pcd_with_ply_ext.ply"), FileFormat::PLY);
    
    // PLY content with PCD extension - content should take priority
    EXPECT_EQ(FileFormatDetector::detectByContent("/tmp/ply_with_pcd_ext.pcd"), FileFormat::PLY);
    EXPECT_EQ(FileFormatDetector::detectByExtension("/tmp/ply_with_pcd_ext.pcd"), FileFormat::PCD);
}

// Test unified format detection (should prefer content over extension)
TEST_F(FileFormatDetectorTest, DetectUnifiedFormat) {
    // Standard cases - extension and content match
    std::string cube_ply = test_data_dir + "/cube.ply";
    EXPECT_EQ(FileFormatDetector::detectFormat(cube_ply), FileFormat::PLY);
    
    // Mixed cases - content should take priority
    EXPECT_EQ(FileFormatDetector::detectFormat("/tmp/pcd_with_ply_ext.ply"), FileFormat::PCD);
    EXPECT_EQ(FileFormatDetector::detectFormat("/tmp/ply_with_pcd_ext.pcd"), FileFormat::PLY);
    
    // Unknown file
    EXPECT_EQ(FileFormatDetector::detectFormat("/tmp/invalid_file.xyz"), FileFormat::UNKNOWN);
    EXPECT_EQ(FileFormatDetector::detectFormat("/nonexistent/file.pcd"), FileFormat::UNKNOWN);
}

// Test utility functions
TEST_F(FileFormatDetectorTest, UtilityFunctions) {
    // Test formatToString
    EXPECT_EQ(FileFormatDetector::formatToString(FileFormat::PCD), "PCD");
    EXPECT_EQ(FileFormatDetector::formatToString(FileFormat::PLY), "PLY");
    EXPECT_EQ(FileFormatDetector::formatToString(FileFormat::UNKNOWN), "UNKNOWN");
    
    // Test isSupportedFormat
    EXPECT_TRUE(FileFormatDetector::isSupportedFormat(FileFormat::PCD));
    EXPECT_TRUE(FileFormatDetector::isSupportedFormat(FileFormat::PLY));
    EXPECT_FALSE(FileFormatDetector::isSupportedFormat(FileFormat::UNKNOWN));
    
    // Test getFileExtension
    EXPECT_EQ(FileFormatDetector::getFileExtension("file.pcd"), ".pcd");
    EXPECT_EQ(FileFormatDetector::getFileExtension("file.PLY"), ".PLY");
    EXPECT_EQ(FileFormatDetector::getFileExtension("path/to/file.ply"), ".ply");
    EXPECT_EQ(FileFormatDetector::getFileExtension("file"), "");
    EXPECT_EQ(FileFormatDetector::getFileExtension(""), "");
    EXPECT_EQ(FileFormatDetector::getFileExtension(".hidden"), "");
    EXPECT_EQ(FileFormatDetector::getFileExtension("file.tar.gz"), ".gz");
}

// Test edge cases and error conditions
TEST_F(FileFormatDetectorTest, EdgeCases) {
    // Empty filename
    EXPECT_EQ(FileFormatDetector::detectFormat(""), FileFormat::UNKNOWN);
    
    // Directory path
    EXPECT_EQ(FileFormatDetector::detectFormat("/tmp"), FileFormat::UNKNOWN);
    
    // File with correct extension but corrupted content
    std::string invalid_ply = test_data_dir + "/invalid.ply";
    // Should still detect as PLY by extension but UNKNOWN by content
    EXPECT_EQ(FileFormatDetector::detectByExtension(invalid_ply), FileFormat::PLY);
    
    // Very long filename
    std::string long_name(1000, 'a');
    long_name += ".pcd";
    EXPECT_EQ(FileFormatDetector::detectByExtension(long_name), FileFormat::PCD);
}

// Test case sensitivity
TEST_F(FileFormatDetectorTest, CaseSensitivity) {
    // Test various case combinations
    EXPECT_EQ(FileFormatDetector::detectByExtension("file.pcd"), FileFormat::PCD);
    EXPECT_EQ(FileFormatDetector::detectByExtension("file.PCD"), FileFormat::PCD);
    EXPECT_EQ(FileFormatDetector::detectByExtension("file.Pcd"), FileFormat::PCD);
    EXPECT_EQ(FileFormatDetector::detectByExtension("file.pCd"), FileFormat::PCD);
    
    EXPECT_EQ(FileFormatDetector::detectByExtension("file.ply"), FileFormat::PLY);
    EXPECT_EQ(FileFormatDetector::detectByExtension("file.PLY"), FileFormat::PLY);
    EXPECT_EQ(FileFormatDetector::detectByExtension("file.Ply"), FileFormat::PLY);
    EXPECT_EQ(FileFormatDetector::detectByExtension("file.pLy"), FileFormat::PLY);
}