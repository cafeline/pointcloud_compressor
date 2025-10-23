// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <filesystem>
#include "pointcloud_compressor/core/PatternEncoder.hpp"

using namespace pointcloud_compressor;

class PatternEncoderTest : public ::testing::Test {
protected:
    void SetUp() override {
        temp_dir = "test_temp";
        std::filesystem::create_directories(temp_dir);
    }
    
    void TearDown() override {
        if (std::filesystem::exists(temp_dir)) {
            std::filesystem::remove_all(temp_dir);
        }
    }
    
    std::vector<uint16_t> createTestIndices8bit() {
        return {0, 1, 2, 0, 1, 255, 100, 50};  // All values < 256
    }
    
    std::vector<uint16_t> createTestIndices16bit() {
        return {0, 1, 256, 1000, 65535, 0, 256};  // Some values >= 256
    }
    
    std::string temp_dir;
};

// Test encoding patterns with 8-bit indices
TEST_F(PatternEncoderTest, EncodePatterns8bit) {
    PatternEncoder encoder;
    auto indices = createTestIndices8bit();
    
    std::string filename = temp_dir + "/test_8bit.bin";
    bool result = encoder.encodePatterns8bit(indices, filename);
    
    EXPECT_TRUE(result);
    EXPECT_TRUE(std::filesystem::exists(filename));
    
    // File should exist and have reasonable size
    auto file_size = std::filesystem::file_size(filename);
    EXPECT_GT(file_size, 0);
    EXPECT_LT(file_size, 1000);  // Should be small for test data
}

// Test encoding patterns with 16-bit indices
TEST_F(PatternEncoderTest, EncodePatterns16bit) {
    PatternEncoder encoder;
    auto indices = createTestIndices16bit();
    
    std::string filename = temp_dir + "/test_16bit.bin";
    bool result = encoder.encodePatterns(indices, filename);
    
    EXPECT_TRUE(result);
    EXPECT_TRUE(std::filesystem::exists(filename));
    
    // File should exist and be larger than 8-bit version (roughly)
    auto file_size = std::filesystem::file_size(filename);
    EXPECT_GT(file_size, 0);
}

// Test decoding patterns
TEST_F(PatternEncoderTest, DecodePatterns) {
    PatternEncoder encoder;
    auto original_indices = createTestIndices8bit();
    
    // Encode first
    std::string filename = temp_dir + "/test_decode.bin";
    bool encode_result = encoder.encodePatterns8bit(original_indices, filename);
    ASSERT_TRUE(encode_result);
    
    // Then decode
    std::vector<uint64_t> decoded_indices;
    bool decode_result = encoder.decodePatterns(filename, decoded_indices);
    
    EXPECT_TRUE(decode_result);
    EXPECT_EQ(decoded_indices.size(), original_indices.size());
    
    // Verify all indices match
    for (size_t i = 0; i < original_indices.size(); ++i) {
        EXPECT_EQ(decoded_indices[i], original_indices[i]);
    }
}

// Test round-trip with 16-bit indices
TEST_F(PatternEncoderTest, RoundTrip16bit) {
    PatternEncoder encoder;
    auto original_indices = createTestIndices16bit();
    
    // Encode
    std::string filename = temp_dir + "/test_roundtrip_16bit.bin";
    bool encode_result = encoder.encodePatterns(original_indices, filename);
    ASSERT_TRUE(encode_result);
    
    // Decode
    std::vector<uint64_t> decoded_indices;
    bool decode_result = encoder.decodePatterns(filename, decoded_indices);
    
    EXPECT_TRUE(decode_result);
    EXPECT_EQ(decoded_indices.size(), original_indices.size());
    
    // Verify all indices match
    for (size_t i = 0; i < original_indices.size(); ++i) {
        EXPECT_EQ(decoded_indices[i], static_cast<uint64_t>(original_indices[i]));
    }
}

// Test canUse8bitIndices function
TEST_F(PatternEncoderTest, CanUse8bitIndicesTrue) {
    PatternEncoder encoder;
    auto indices = createTestIndices8bit();
    
    bool result = encoder.canUse8bitIndices(indices);
    
    EXPECT_TRUE(result);
}

// Test canUse8bitIndices function with 16-bit values
TEST_F(PatternEncoderTest, CanUse8bitIndicesFalse) {
    PatternEncoder encoder;
    auto indices = createTestIndices16bit();
    
    bool result = encoder.canUse8bitIndices(indices);
    
    EXPECT_FALSE(result);
}

// Test encoding empty indices
TEST_F(PatternEncoderTest, EncodeEmptyIndices) {
    PatternEncoder encoder;
    std::vector<uint16_t> empty_indices;
    
    std::string filename = temp_dir + "/test_empty.bin";
    bool result = encoder.encodePatterns(empty_indices, filename);
    
    EXPECT_TRUE(result);
    EXPECT_TRUE(std::filesystem::exists(filename));
}

// Test decoding empty indices
TEST_F(PatternEncoderTest, DecodeEmptyIndices) {
    PatternEncoder encoder;
    std::vector<uint16_t> empty_indices;
    
    // Encode empty indices first
    std::string filename = temp_dir + "/test_empty_decode.bin";
    bool encode_result = encoder.encodePatterns(empty_indices, filename);
    ASSERT_TRUE(encode_result);
    
    // Decode
    std::vector<uint64_t> decoded_indices;
    bool decode_result = encoder.decodePatterns(filename, decoded_indices);
    
    EXPECT_TRUE(decode_result);
    EXPECT_EQ(decoded_indices.size(), 0);
}

// Test encoding to invalid path
TEST_F(PatternEncoderTest, EncodeToInvalidPath) {
    PatternEncoder encoder;
    auto indices = createTestIndices8bit();
    
    std::string invalid_path = "/invalid/path/file.bin";
    bool result = encoder.encodePatterns(indices, invalid_path);
    
    EXPECT_FALSE(result);
}

// Test decoding from non-existent file
TEST_F(PatternEncoderTest, DecodeNonExistentFile) {
    PatternEncoder encoder;
    std::vector<uint64_t> indices;
    
    bool result = encoder.decodePatterns("non_existent_file.bin", indices);
    
    EXPECT_FALSE(result);
}

// Test 8-bit encoding with values too large
TEST_F(PatternEncoderTest, Encode8bitWithLargeValues) {
    PatternEncoder encoder;
    auto indices = createTestIndices16bit();  // Contains values >= 256
    
    std::string filename = temp_dir + "/test_8bit_large.bin";
    bool result = encoder.encodePatterns8bit(indices, filename);
    
    // Should fail because canUse8bitIndices returns false
    EXPECT_FALSE(result);
}

// Test boundary values for 8-bit encoding
TEST_F(PatternEncoderTest, BoundaryValues8bit) {
    PatternEncoder encoder;
    std::vector<uint16_t> boundary_indices = {0, 255, 254, 1};
    
    // Should be able to use 8-bit
    EXPECT_TRUE(encoder.canUse8bitIndices(boundary_indices));
    
    std::string filename = temp_dir + "/test_boundary.bin";
    bool encode_result = encoder.encodePatterns8bit(boundary_indices, filename);
    EXPECT_TRUE(encode_result);
    
    // Verify round-trip
    std::vector<uint64_t> decoded_indices;
    bool decode_result = encoder.decodePatterns(filename, decoded_indices);
    EXPECT_TRUE(decode_result);
    
    // Convert boundary_indices to uint64_t for comparison
    std::vector<uint64_t> expected_indices(boundary_indices.begin(), boundary_indices.end());
    EXPECT_EQ(decoded_indices, expected_indices);
}

// Test with one value just over 8-bit limit
TEST_F(PatternEncoderTest, JustOver8bitLimit) {
    PatternEncoder encoder;
    std::vector<uint16_t> indices = {0, 255, 256};  // 256 is just over 8-bit limit
    
    EXPECT_FALSE(encoder.canUse8bitIndices(indices));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}