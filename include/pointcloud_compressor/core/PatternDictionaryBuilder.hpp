// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_PATTERN_DICTIONARY_BUILDER_HPP
#define POINTCLOUD_COMPRESSOR_PATTERN_DICTIONARY_BUILDER_HPP

#include <vector>
#include <map>
#include <string>
#include <cstdint>
#include <limits>

namespace pointcloud_compressor {

class PatternDictionaryBuilder {
public:
    PatternDictionaryBuilder();
    ~PatternDictionaryBuilder();
    
    // Build dictionary from patterns
    bool buildDictionary(const std::vector<std::vector<uint8_t>>& patterns);
    
    // Get unique patterns
    const std::vector<std::vector<uint8_t>>& getUniquePatterns() const;
    
    // Get pattern indices as 64-bit (caller will downcast as needed)
    const std::vector<uint64_t>& getPatternIndices() const;
    
    // Get required index bit size (8, 16, 32, or 64)
    int getRequiredIndexBitSize() const;
    
    // Get maximum index value
    uint64_t getMaxIndex() const { return max_index_; }
    
    // Save/load dictionary with specified index bit size
    bool saveDictionary(const std::string& filename, int index_bit_size = 0) const;
    bool loadDictionary(const std::string& filename);

    // Statistics
    size_t getUniquePatternCount() const;
    float getCompressionRatio() const;

    // Load dictionary directly from pattern payloads (used for archive playback)
    void setDictionary(const std::vector<std::vector<uint8_t>>& patterns);
    
private:
    std::vector<std::vector<uint8_t>> unique_patterns_;
    std::vector<uint64_t> pattern_indices_;  // Store as 64-bit internally
    std::map<std::vector<uint8_t>, uint64_t> pattern_to_index_;
    uint64_t max_index_ = 0;
};

} // namespace pointcloud_compressor

#endif // POINTCLOUD_COMPRESSOR_PATTERN_DICTIONARY_BUILDER_HPP
