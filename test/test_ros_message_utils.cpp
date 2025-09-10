#include <gtest/gtest.h>
#include "pointcloud_compressor/ros/MessageUtils.hpp"
#include <algorithm>

using namespace pointcloud_compressor;
using namespace pointcloud_compressor::ros;

class MessageUtilsTest : public ::testing::Test {
protected:
    void SetUp() override {
    }
};

TEST_F(MessageUtilsTest, PackAndExtract8BitIndices) {
    // Test with indices that fit in 8 bits
    std::vector<uint16_t> original_indices = {0, 10, 50, 100, 200, 255};
    
    uint8_t index_bit_size;
    std::vector<uint8_t> packed_data;
    
    // Pack indices
    MessageUtils::packBlockIndices(original_indices, index_bit_size, packed_data);
    
    // Should use 8-bit encoding
    EXPECT_EQ(index_bit_size, 8);
    EXPECT_EQ(packed_data.size(), original_indices.size());
    
    // Create message with packed data
    pointcloud_compressor::msg::PatternDictionary msg;
    msg.index_bit_size = index_bit_size;
    msg.block_indices_data = packed_data;
    
    // Extract indices
    std::vector<uint16_t> extracted_indices = MessageUtils::extractBlockIndices(msg);
    
    // Should match original
    EXPECT_EQ(extracted_indices, original_indices);
}

TEST_F(MessageUtilsTest, PackAndExtract16BitIndices) {
    // Test with indices that require 16 bits
    std::vector<uint16_t> original_indices = {0, 100, 256, 1000, 5000, 65535};
    
    uint8_t index_bit_size;
    std::vector<uint8_t> packed_data;
    
    // Pack indices
    MessageUtils::packBlockIndices(original_indices, index_bit_size, packed_data);
    
    // Should use 16-bit encoding
    EXPECT_EQ(index_bit_size, 16);
    EXPECT_EQ(packed_data.size(), original_indices.size() * 2);
    
    // Create message with packed data
    pointcloud_compressor::msg::PatternDictionary msg;
    msg.index_bit_size = index_bit_size;
    msg.block_indices_data = packed_data;
    
    // Extract indices
    std::vector<uint16_t> extracted_indices = MessageUtils::extractBlockIndices(msg);
    
    // Should match original
    EXPECT_EQ(extracted_indices, original_indices);
}

TEST_F(MessageUtilsTest, BoundaryCase256) {
    // Test boundary at 256
    {
        std::vector<uint16_t> indices_255 = {0, 100, 200, 255};
        uint8_t index_bit_size;
        std::vector<uint8_t> packed_data;
        
        MessageUtils::packBlockIndices(indices_255, index_bit_size, packed_data);
        EXPECT_EQ(index_bit_size, 8);
    }
    
    {
        std::vector<uint16_t> indices_256 = {0, 100, 200, 256};
        uint8_t index_bit_size;
        std::vector<uint8_t> packed_data;
        
        MessageUtils::packBlockIndices(indices_256, index_bit_size, packed_data);
        EXPECT_EQ(index_bit_size, 16);
    }
}

TEST_F(MessageUtilsTest, EmptyIndices) {
    std::vector<uint16_t> empty_indices;
    
    uint8_t index_bit_size;
    std::vector<uint8_t> packed_data;
    
    MessageUtils::packBlockIndices(empty_indices, index_bit_size, packed_data);
    
    // Should default to 8-bit for empty
    EXPECT_EQ(index_bit_size, 8);
    EXPECT_TRUE(packed_data.empty());
    
    // Test extraction
    pointcloud_compressor::msg::PatternDictionary msg;
    msg.index_bit_size = index_bit_size;
    msg.block_indices_data = packed_data;
    
    std::vector<uint16_t> extracted = MessageUtils::extractBlockIndices(msg);
    EXPECT_TRUE(extracted.empty());
}

TEST_F(MessageUtilsTest, CalculateSavings) {
    // Test with 8-bit compatible indices
    {
        std::vector<uint16_t> small_indices = {0, 10, 50, 100, 200, 255};
        size_t savings = MessageUtils::calculateIndexEncodingSavings(small_indices);
        EXPECT_EQ(savings, small_indices.size()); // 1 byte saved per index
    }
    
    // Test with 16-bit required indices
    {
        std::vector<uint16_t> large_indices = {0, 100, 256, 1000, 5000};
        size_t savings = MessageUtils::calculateIndexEncodingSavings(large_indices);
        EXPECT_EQ(savings, 0); // No savings
    }
    
    // Test empty
    {
        std::vector<uint16_t> empty;
        size_t savings = MessageUtils::calculateIndexEncodingSavings(empty);
        EXPECT_EQ(savings, 0);
    }
}

TEST_F(MessageUtilsTest, LargeDataSet) {
    // Test with large dataset that benefits from 8-bit encoding
    std::vector<uint16_t> original_indices;
    for (int i = 0; i < 10000; ++i) {
        original_indices.push_back(i % 200); // All values < 256
    }
    
    uint8_t index_bit_size;
    std::vector<uint8_t> packed_data;
    
    MessageUtils::packBlockIndices(original_indices, index_bit_size, packed_data);
    
    // Should use 8-bit encoding
    EXPECT_EQ(index_bit_size, 8);
    EXPECT_EQ(packed_data.size(), original_indices.size());
    
    // Calculate savings
    size_t savings = MessageUtils::calculateIndexEncodingSavings(original_indices);
    EXPECT_EQ(savings, 10000); // 10000 bytes saved
    
    // Test round-trip
    pointcloud_compressor::msg::PatternDictionary msg;
    msg.index_bit_size = index_bit_size;
    msg.block_indices_data = packed_data;
    
    std::vector<uint16_t> extracted = MessageUtils::extractBlockIndices(msg);
    EXPECT_EQ(extracted, original_indices);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}