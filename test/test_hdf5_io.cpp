#include <gtest/gtest.h>
#include <pointcloud_compressor/io/HDF5IO.hpp>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <fstream>

using namespace pointcloud_compressor;

class HDF5IOTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test data
        test_data_ = createTestData();
        
        // Create temporary test file path
        test_file_ = "/tmp/test_compressed_map_" + 
                     std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + 
                     ".h5";
    }
    
    void TearDown() override {
        // Clean up test file
        if (std::filesystem::exists(test_file_)) {
            std::filesystem::remove(test_file_);
        }
    }
    
    CompressedMapData createTestData() {
        CompressedMapData data;
        
        // Set metadata
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::gmtime(&time_t), "%Y-%m-%dT%H:%M:%SZ");
        data.creation_time = ss.str();
        
        // Set compression parameters
        data.voxel_size = 0.1f;
        data.dictionary_size = 256;
        data.pattern_bits = 8;
        data.block_size = 8;
        
        // Create test dictionary patterns
        // Each pattern is 8x8x8 = 512 bits = 64 bytes
        data.pattern_length = 512;
        data.dictionary_patterns.resize(data.dictionary_size * 64);
        for (size_t i = 0; i < data.dictionary_patterns.size(); ++i) {
            data.dictionary_patterns[i] = static_cast<uint8_t>(i % 256);
        }
        
        // Create test compressed data
        data.voxel_indices = {0, 1, 2, 3, 4, 5, 10, 20, 30, 40, 50, 100, 150, 200, 255};
        data.voxel_positions = {
            {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1},
            {1, 1, 0}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1},
            {2, 0, 0}, {0, 2, 0}, {0, 0, 2}, {2, 2, 0},
            {2, 0, 2}, {0, 2, 2}, {2, 2, 2}
        };
        
        // Set statistics
        data.original_points = 10000;
        data.compressed_voxels = data.voxel_positions.size();
        data.compression_ratio = static_cast<double>(data.original_points) / 
                                static_cast<double>(data.compressed_voxels);
        data.bounding_box_min = {-10.0, -10.0, -10.0};
        data.bounding_box_max = {10.0, 10.0, 10.0};
        
        return data;
    }
    
    bool compareData(const CompressedMapData& a, const CompressedMapData& b) {
        // Compare metadata
        if (a.version != b.version || 
            a.frame_id != b.frame_id ||
            a.compression_method != b.compression_method) {
            return false;
        }
        
        // Compare parameters
        if (std::abs(a.voxel_size - b.voxel_size) > 1e-6 ||
            a.dictionary_size != b.dictionary_size ||
            a.pattern_bits != b.pattern_bits ||
            a.block_size != b.block_size) {
            return false;
        }
        
        // Compare dictionary
        if (a.pattern_length != b.pattern_length ||
            a.dictionary_patterns != b.dictionary_patterns) {
            return false;
        }
        
        // Compare compressed data
        if (a.voxel_indices != b.voxel_indices ||
            a.voxel_positions != b.voxel_positions) {
            return false;
        }
        
        // Compare statistics
        if (a.original_points != b.original_points ||
            a.compressed_voxels != b.compressed_voxels ||
            std::abs(a.compression_ratio - b.compression_ratio) > 1e-6) {
            return false;
        }
        
        // Compare bounding box
        for (int i = 0; i < 3; ++i) {
            if (std::abs(a.bounding_box_min[i] - b.bounding_box_min[i]) > 1e-6 ||
                std::abs(a.bounding_box_max[i] - b.bounding_box_max[i]) > 1e-6) {
                return false;
            }
        }
        
        return true;
    }
    
    CompressedMapData test_data_;
    std::string test_file_;
};

TEST_F(HDF5IOTest, WriteAndReadBasic) {
    HDF5IO io;
    
    // Write test data
    ASSERT_TRUE(io.write(test_file_, test_data_));
    
    // Verify file exists
    ASSERT_TRUE(std::filesystem::exists(test_file_));
    
    // Read data back
    CompressedMapData read_data;
    ASSERT_TRUE(io.read(test_file_, read_data));
    
    // Compare data
    EXPECT_TRUE(compareData(test_data_, read_data));
}

TEST_F(HDF5IOTest, IsValidHDF5) {
    HDF5IO io;
    
    // Non-existent file
    EXPECT_FALSE(io.isValidHDF5("/tmp/non_existent_file.h5"));
    
    // Write valid HDF5 file
    ASSERT_TRUE(io.write(test_file_, test_data_));
    EXPECT_TRUE(io.isValidHDF5(test_file_));
    
    // Create invalid file
    std::string invalid_file = "/tmp/invalid_file.h5";
    std::ofstream ofs(invalid_file);
    ofs << "This is not an HDF5 file";
    ofs.close();
    EXPECT_FALSE(io.isValidHDF5(invalid_file));
    std::filesystem::remove(invalid_file);
}

TEST_F(HDF5IOTest, ReadNonExistentFile) {
    HDF5IO io;
    CompressedMapData data;
    
    EXPECT_FALSE(io.read("/tmp/non_existent_file.h5", data));
    EXPECT_FALSE(io.getLastError().empty());
}

TEST_F(HDF5IOTest, WriteToInvalidPath) {
    HDF5IO io;
    
    // Try to write to invalid directory
    EXPECT_FALSE(io.write("/invalid/path/file.h5", test_data_));
    EXPECT_FALSE(io.getLastError().empty());
}

TEST_F(HDF5IOTest, LargeDataset) {
    HDF5IO io;
    CompressedMapData large_data = test_data_;
    
    // Create large dataset
    size_t num_voxels = 100000;
    large_data.voxel_indices.clear();
    large_data.voxel_positions.clear();
    
    for (size_t i = 0; i < num_voxels; ++i) {
        large_data.voxel_indices.push_back(static_cast<uint16_t>(i % 256));
        large_data.voxel_positions.push_back({
            static_cast<int32_t>(i % 100),
            static_cast<int32_t>((i / 100) % 100),
            static_cast<int32_t>(i / 10000)
        });
    }
    
    large_data.compressed_voxels = num_voxels;
    
    // Write and read
    ASSERT_TRUE(io.write(test_file_, large_data));
    
    CompressedMapData read_data;
    ASSERT_TRUE(io.read(test_file_, read_data));
    
    // Verify sizes
    EXPECT_EQ(read_data.voxel_indices.size(), num_voxels);
    EXPECT_EQ(read_data.voxel_positions.size(), num_voxels);
    EXPECT_TRUE(compareData(large_data, read_data));
}

TEST_F(HDF5IOTest, EmptyData) {
    HDF5IO io;
    CompressedMapData empty_data;
    
    // Set minimal required fields
    empty_data.creation_time = "2024-01-01T00:00:00Z";
    
    // Write and read empty data
    ASSERT_TRUE(io.write(test_file_, empty_data));
    
    CompressedMapData read_data;
    ASSERT_TRUE(io.read(test_file_, read_data));
    
    // Verify empty data
    EXPECT_TRUE(read_data.voxel_indices.empty());
    EXPECT_TRUE(read_data.voxel_positions.empty());
    EXPECT_TRUE(read_data.dictionary_patterns.empty());
}

TEST_F(HDF5IOTest, MetadataPreservation) {
    HDF5IO io;
    
    // Set specific metadata
    test_data_.frame_id = "custom_frame";
    test_data_.compression_method = "advanced_voxel";
    test_data_.version = "2.0.0";
    
    // Write and read
    ASSERT_TRUE(io.write(test_file_, test_data_));
    
    CompressedMapData read_data;
    ASSERT_TRUE(io.read(test_file_, read_data));
    
    // Verify metadata
    EXPECT_EQ(read_data.frame_id, "custom_frame");
    EXPECT_EQ(read_data.compression_method, "advanced_voxel");
    EXPECT_EQ(read_data.version, "2.0.0");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}