#include <gtest/gtest.h>
#include <filesystem>
#include "pointcloud_compressor/core/PatternDictionaryBuilder.hpp"

using namespace pointcloud_compressor;

class DictionaryBuilderTest : public ::testing::Test {
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
    
    std::vector<std::vector<uint8_t>> createTestPatterns() {
        std::vector<std::vector<uint8_t>> patterns;
        
        // Pattern 1: [1, 2, 3, 4]
        patterns.push_back({1, 2, 3, 4});
        
        // Pattern 2: [5, 6, 7, 8] 
        patterns.push_back({5, 6, 7, 8});
        
        // Pattern 1 again (duplicate)
        patterns.push_back({1, 2, 3, 4});
        
        // Pattern 3: [9, 10, 11, 12]
        patterns.push_back({9, 10, 11, 12});
        
        // Pattern 2 again (duplicate)
        patterns.push_back({5, 6, 7, 8});
        
        return patterns;
    }
    
    std::string temp_dir;
};

// Test building empty dictionary
TEST_F(DictionaryBuilderTest, BuildEmptyDictionary) {
    PatternDictionaryBuilder builder;
    std::vector<std::vector<uint8_t>> empty_patterns;
    
    bool result = builder.buildDictionary(empty_patterns);
    
    EXPECT_FALSE(result);
    EXPECT_EQ(builder.getUniquePatternCount(), 0);
}

// Test detecting duplicate patterns
TEST_F(DictionaryBuilderTest, DetectDuplicatePatterns) {
    PatternDictionaryBuilder builder;
    auto patterns = createTestPatterns();
    
    bool result = builder.buildDictionary(patterns);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(builder.getUniquePatternCount(), 3);  // 3 unique patterns
    
    const auto& indices = builder.getPatternIndices();
    EXPECT_EQ(indices.size(), 5);  // 5 total patterns
    
    // Check index mapping
    EXPECT_EQ(indices[0], indices[2]);  // First and third should be same (pattern 1)
    EXPECT_EQ(indices[1], indices[4]);  // Second and fifth should be same (pattern 2)
    EXPECT_NE(indices[0], indices[1]);  // Different patterns should have different indices
    EXPECT_NE(indices[0], indices[3]);  // Different patterns should have different indices
}

// Test serializing dictionary
TEST_F(DictionaryBuilderTest, SerializeDictionary) {
    PatternDictionaryBuilder builder;
    auto patterns = createTestPatterns();
    
    bool build_result = builder.buildDictionary(patterns);
    ASSERT_TRUE(build_result);
    
    std::string filename = temp_dir + "/test_dict.bin";
    bool save_result = builder.saveDictionary(filename);
    
    EXPECT_TRUE(save_result);
    EXPECT_TRUE(std::filesystem::exists(filename));
}

// Test deserializing dictionary
TEST_F(DictionaryBuilderTest, DeserializeDictionary) {
    // First, create and save a dictionary
    PatternDictionaryBuilder builder1;
    auto patterns = createTestPatterns();
    builder1.buildDictionary(patterns);
    
    std::string filename = temp_dir + "/test_dict.bin";
    bool save_result = builder1.saveDictionary(filename);
    ASSERT_TRUE(save_result);
    
    // Then, load it with a new builder
    PatternDictionaryBuilder builder2;
    bool load_result = builder2.loadDictionary(filename);
    
    EXPECT_TRUE(load_result);
    EXPECT_EQ(builder2.getUniquePatternCount(), 3);
    
    // Verify patterns are the same
    const auto& original_patterns = builder1.getUniquePatterns();
    const auto& loaded_patterns = builder2.getUniquePatterns();
    
    ASSERT_EQ(original_patterns.size(), loaded_patterns.size());
    for (size_t i = 0; i < original_patterns.size(); ++i) {
        EXPECT_EQ(original_patterns[i], loaded_patterns[i]);
    }
}

// Test compression ratio calculation
TEST_F(DictionaryBuilderTest, CalculateCompressionRatio) {
    PatternDictionaryBuilder builder;
    auto patterns = createTestPatterns();
    
    bool result = builder.buildDictionary(patterns);
    ASSERT_TRUE(result);
    
    float ratio = builder.getCompressionRatio();
    EXPECT_GT(ratio, 0.0f);
    EXPECT_LE(ratio, 1.0f);  // Should be less than or equal to 1 (compressed)
}

// Test building dictionary with single pattern
TEST_F(DictionaryBuilderTest, SinglePattern) {
    PatternDictionaryBuilder builder;
    std::vector<std::vector<uint8_t>> single_pattern = {{1, 2, 3, 4}};
    
    bool result = builder.buildDictionary(single_pattern);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(builder.getUniquePatternCount(), 1);
    
    const auto& indices = builder.getPatternIndices();
    EXPECT_EQ(indices.size(), 1);
    EXPECT_EQ(indices[0], 0);
}

// Test building dictionary with all unique patterns
TEST_F(DictionaryBuilderTest, AllUniquePatterns) {
    PatternDictionaryBuilder builder;
    std::vector<std::vector<uint8_t>> unique_patterns = {
        {1, 2, 3, 4},
        {5, 6, 7, 8},
        {9, 10, 11, 12}
    };
    
    bool result = builder.buildDictionary(unique_patterns);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(builder.getUniquePatternCount(), 3);
    
    const auto& indices = builder.getPatternIndices();
    EXPECT_EQ(indices.size(), 3);
    
    // All indices should be different
    EXPECT_NE(indices[0], indices[1]);
    EXPECT_NE(indices[0], indices[2]);
    EXPECT_NE(indices[1], indices[2]);
}

// Test loading non-existent dictionary file
TEST_F(DictionaryBuilderTest, LoadNonExistentFile) {
    PatternDictionaryBuilder builder;
    
    bool result = builder.loadDictionary("non_existent_file.bin");
    
    EXPECT_FALSE(result);
    EXPECT_EQ(builder.getUniquePatternCount(), 0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}