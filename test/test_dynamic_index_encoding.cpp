#include <gtest/gtest.h>
#include "pointcloud_compressor/core/PatternEncoder.hpp"
#include <vector>
#include <fstream>

using namespace pointcloud_compressor;

class DynamicIndexEncodingTest : public ::testing::Test {
protected:
    void SetUp() override {
        encoder_ = std::make_unique<PatternEncoder>();
    }

    std::unique_ptr<PatternEncoder> encoder_;
    
    // Helper function to check file size
    size_t getFileSize(const std::string& filename) {
        std::ifstream file(filename, std::ios::binary | std::ios::ate);
        if (!file.is_open()) return 0;
        return file.tellg();
    }
    
    // Helper function to read the index size from encoded file
    uint8_t readIndexSizeFromFile(const std::string& filename) {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) return 0;
        
        uint32_t num_indices;
        uint8_t index_size;
        
        file.read(reinterpret_cast<char*>(&num_indices), sizeof(num_indices));
        file.read(reinterpret_cast<char*>(&index_size), sizeof(index_size));
        
        return index_size;
    }
};

TEST_F(DynamicIndexEncodingTest, AutoSelectsIndexSize) {
    // Test automatic selection based on max index value
    
    // Case 1: Small indices (max < 256) should use 8-bit
    {
        std::vector<uint64_t> small_indices;
        for (uint64_t i = 0; i < 100; ++i) {
            small_indices.push_back(i % 200);  // Max value is 199
        }
        
        std::string test_file = "/tmp/test_auto_8bit.bin";
        EXPECT_TRUE(encoder_->encodePatternsAuto(small_indices, test_file));
        
        // Check that 8-bit encoding was used
        EXPECT_EQ(readIndexSizeFromFile(test_file), 8);
        
        // Verify decoding works
        std::vector<uint64_t> decoded;
        EXPECT_TRUE(encoder_->decodePatterns(test_file, decoded));
        EXPECT_EQ(small_indices, decoded);
    }
    
    // Case 2: Large indices (max >= 256) should use 16-bit
    {
        std::vector<uint64_t> large_indices;
        for (uint64_t i = 0; i < 100; ++i) {
            large_indices.push_back(i * 10);  // Max value is 990
        }
        
        std::string test_file = "/tmp/test_auto_16bit.bin";
        EXPECT_TRUE(encoder_->encodePatternsAuto(large_indices, test_file));
        
        // Check that 16-bit encoding was used
        EXPECT_EQ(readIndexSizeFromFile(test_file), 16);
        
        // Verify decoding works
        std::vector<uint64_t> decoded;
        EXPECT_TRUE(encoder_->decodePatterns(test_file, decoded));
        EXPECT_EQ(large_indices, decoded);
    }
}

TEST_F(DynamicIndexEncodingTest, FileSizeOptimization) {
    // Test that 8-bit encoding produces smaller files for small indices
    
    std::vector<uint64_t> indices;
    for (uint64_t i = 0; i < 1000; ++i) {
        indices.push_back(i % 100);  // All values < 256
    }
    
    std::string file_8bit = "/tmp/test_size_8bit.bin";
    std::string file_16bit = "/tmp/test_size_16bit.bin";
    
    // Encode with auto (should use 8-bit)
    EXPECT_TRUE(encoder_->encodePatternsAuto(indices, file_8bit));
    
    // Force 16-bit encoding
    EXPECT_TRUE(encoder_->encodePatternsWithBitSize(indices, file_16bit, 16));
    
    // 8-bit file should be smaller
    size_t size_8bit = getFileSize(file_8bit);
    size_t size_16bit = getFileSize(file_16bit);
    
    EXPECT_LT(size_8bit, size_16bit);
    // Expected difference is about 1000 bytes (1000 indices * 1 byte saved)
    EXPECT_EQ(size_16bit - size_8bit, 1000);
}

TEST_F(DynamicIndexEncodingTest, BoundaryCase256) {
    // Test the boundary at exactly 256
    
    // Case 1: Max index 255 -> should use 8-bit
    {
        std::vector<uint64_t> indices_255 = {0, 100, 200, 255};
        std::string test_file = "/tmp/test_boundary_255.bin";
        
        EXPECT_TRUE(encoder_->encodePatternsAuto(indices_255, test_file));
        EXPECT_EQ(readIndexSizeFromFile(test_file), 8);
        
        std::vector<uint64_t> decoded;
        EXPECT_TRUE(encoder_->decodePatterns(test_file, decoded));
        EXPECT_EQ(indices_255, decoded);
    }
    
    // Case 2: Max index 256 -> should use 16-bit
    {
        std::vector<uint64_t> indices_256 = {0, 100, 200, 256};
        std::string test_file = "/tmp/test_boundary_256.bin";
        
        EXPECT_TRUE(encoder_->encodePatternsAuto(indices_256, test_file));
        EXPECT_EQ(readIndexSizeFromFile(test_file), 16);
        
        std::vector<uint64_t> decoded;
        EXPECT_TRUE(encoder_->decodePatterns(test_file, decoded));
        EXPECT_EQ(indices_256, decoded);
    }
}

TEST_F(DynamicIndexEncodingTest, EmptyIndices) {
    // Test handling of empty indices
    std::vector<uint64_t> empty_indices;
    std::string test_file = "/tmp/test_empty.bin";
    
    EXPECT_TRUE(encoder_->encodePatternsAuto(empty_indices, test_file));
    
    std::vector<uint64_t> decoded;
    EXPECT_TRUE(encoder_->decodePatterns(test_file, decoded));
    EXPECT_TRUE(decoded.empty());
}

TEST_F(DynamicIndexEncodingTest, SingleIndex) {
    // Test with single index
    
    // Small value
    {
        std::vector<uint64_t> single_small = {42};
        std::string test_file = "/tmp/test_single_small.bin";
        
        EXPECT_TRUE(encoder_->encodePatternsAuto(single_small, test_file));
        EXPECT_EQ(readIndexSizeFromFile(test_file), 8);
        
        std::vector<uint64_t> decoded;
        EXPECT_TRUE(encoder_->decodePatterns(test_file, decoded));
        EXPECT_EQ(single_small, decoded);
    }
    
    // Large value
    {
        std::vector<uint64_t> single_large = {1000};
        std::string test_file = "/tmp/test_single_large.bin";
        
        EXPECT_TRUE(encoder_->encodePatternsAuto(single_large, test_file));
        EXPECT_EQ(readIndexSizeFromFile(test_file), 16);
        
        std::vector<uint64_t> decoded;
        EXPECT_TRUE(encoder_->decodePatterns(test_file, decoded));
        EXPECT_EQ(single_large, decoded);
    }
}

TEST_F(DynamicIndexEncodingTest, MixedRangeOptimization) {
    // Test with mostly small values and one large value
    std::vector<uint64_t> mixed_indices;
    
    // Add 999 small values
    for (int i = 0; i < 999; ++i) {
        mixed_indices.push_back(i % 100);  // All < 100
    }
    
    // Add one large value that forces 16-bit
    mixed_indices.push_back(500);
    
    std::string test_file = "/tmp/test_mixed.bin";
    EXPECT_TRUE(encoder_->encodePatternsAuto(mixed_indices, test_file));
    
    // Should use 16-bit because of the one large value
    EXPECT_EQ(readIndexSizeFromFile(test_file), 16);
    
    std::vector<uint64_t> decoded;
    EXPECT_TRUE(encoder_->decodePatterns(test_file, decoded));
    EXPECT_EQ(mixed_indices, decoded);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}