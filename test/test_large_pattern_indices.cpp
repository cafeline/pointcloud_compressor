#include <gtest/gtest.h>
#include "pointcloud_compressor/core/PatternEncoder.hpp"
#include "pointcloud_compressor/core/PatternDictionaryBuilder.hpp"
#include <vector>
#include <random>

using namespace pointcloud_compressor;

class LargePatternIndicesTest : public ::testing::Test {
protected:
    void SetUp() override {
        encoder_ = std::make_unique<PatternEncoder>();
        dictionary_builder_ = std::make_unique<PatternDictionaryBuilder>();
    }

    std::unique_ptr<PatternEncoder> encoder_;
    std::unique_ptr<PatternDictionaryBuilder> dictionary_builder_;
};

TEST_F(LargePatternIndicesTest, TestMoreThan256Patterns) {
    // Create a vector with indices > 255
    std::vector<uint16_t> indices;
    for (uint16_t i = 0; i < 512; ++i) {
        indices.push_back(i);
    }
    
    // Test that we cannot use 8-bit encoding for indices > 255
    EXPECT_FALSE(encoder_->canUse8bitIndices(indices));
    
    // Test that 16-bit encoding works
    std::string test_file = "/tmp/test_large_indices.bin";
    EXPECT_TRUE(encoder_->encodePatterns(indices, test_file));
    
    // Test decoding
    std::vector<uint64_t> decoded_indices;
    EXPECT_TRUE(encoder_->decodePatterns(test_file, decoded_indices));
    
    // Verify the decoded indices match
    ASSERT_EQ(indices.size(), decoded_indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        EXPECT_EQ(indices[i], decoded_indices[i]) << "Mismatch at index " << i;
    }
}

TEST_F(LargePatternIndicesTest, TestExactly256Patterns) {
    // Test boundary case - indices 0-255 can fit in 8-bit
    std::vector<uint16_t> indices_255;
    for (uint16_t i = 0; i < 256; ++i) {
        indices_255.push_back(i);  // 0 to 255
    }
    // Max value is 255, which fits in uint8_t
    EXPECT_TRUE(encoder_->canUse8bitIndices(indices_255));
    
    // But if we have index 256, it cannot use 8-bit
    std::vector<uint16_t> indices_256;
    for (uint16_t i = 0; i <= 256; ++i) {
        indices_256.push_back(i);  // 0 to 256
    }
    // Max value is 256, which doesn't fit in uint8_t
    EXPECT_FALSE(encoder_->canUse8bitIndices(indices_256));
}

TEST_F(LargePatternIndicesTest, TestVeryLargeIndices) {
    // Test with indices near the 16-bit limit
    std::vector<uint16_t> indices;
    indices.push_back(0);
    indices.push_back(1000);
    indices.push_back(10000);
    indices.push_back(30000);
    indices.push_back(65535);  // Max uint16_t
    
    // Cannot use 8-bit encoding
    EXPECT_FALSE(encoder_->canUse8bitIndices(indices));
    
    // Test 16-bit encoding and decoding
    std::string test_file = "/tmp/test_very_large_indices.bin";
    EXPECT_TRUE(encoder_->encodePatterns(indices, test_file));
    
    std::vector<uint64_t> decoded_indices;
    EXPECT_TRUE(encoder_->decodePatterns(test_file, decoded_indices));
    
    ASSERT_EQ(indices.size(), decoded_indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        EXPECT_EQ(indices[i], decoded_indices[i]);
    }
}

TEST_F(LargePatternIndicesTest, TestPatternDictionaryWithManyPatterns) {
    // Create many unique patterns (more than 256)
    std::vector<std::vector<uint8_t>> patterns;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 1);
    
    // Create 300 unique random patterns
    for (int i = 0; i < 300; ++i) {
        std::vector<uint8_t> pattern(64);  // 8x8x8 = 512 bits = 64 bytes
        for (size_t j = 0; j < pattern.size(); ++j) {
            pattern[j] = 0;
            for (int bit = 0; bit < 8; ++bit) {
                if (dis(gen)) {
                    pattern[j] |= (1 << bit);
                }
            }
        }
        patterns.push_back(pattern);
    }
    
    // Build dictionary
    EXPECT_TRUE(dictionary_builder_->buildDictionary(patterns));
    
    // Get indices - should have values from 0 to 299
    auto indices = dictionary_builder_->getPatternIndices();
    EXPECT_EQ(indices.size(), 300u);
    
    // Check that we have indices > 255
    auto max_index = *std::max_element(indices.begin(), indices.end());
    EXPECT_GT(max_index, 255u);
    
    // Save and load dictionary
    std::string dict_file = "/tmp/test_large_dictionary.bin";
    EXPECT_TRUE(dictionary_builder_->saveDictionary(dict_file));
    
    // Create new dictionary builder and load
    auto new_builder = std::make_unique<PatternDictionaryBuilder>();
    EXPECT_TRUE(new_builder->loadDictionary(dict_file));
    
    // Verify the number of unique patterns
    EXPECT_EQ(new_builder->getUniquePatternCount(), dictionary_builder_->getUniquePatternCount());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}