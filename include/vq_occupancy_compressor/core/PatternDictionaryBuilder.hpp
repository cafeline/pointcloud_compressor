// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_PATTERN_DICTIONARY_BUILDER_HPP
#define VQ_OCCUPANCY_COMPRESSOR_PATTERN_DICTIONARY_BUILDER_HPP

#include <vector>
#include <map>
#include <string>
#include <cstdint>
#include <limits>

namespace vq_occupancy_compressor {

class PatternDictionaryBuilder {
public:
    PatternDictionaryBuilder();
    ~PatternDictionaryBuilder();

    bool buildDictionary(const std::vector<std::vector<uint8_t>>& patterns);
    const std::vector<std::vector<uint8_t>>& getUniquePatterns() const;
    const std::vector<uint64_t>& getPatternIndices() const;
    int getRequiredIndexBitSize() const;
    uint64_t getMaxIndex() const { return max_index_; }
    size_t getUniquePatternCount() const;
    float getCompressionRatio() const;

private:
    std::vector<std::vector<uint8_t>> unique_patterns_;
    std::vector<uint64_t> pattern_indices_;
    std::map<std::vector<uint8_t>, uint64_t> pattern_to_index_;
    uint64_t max_index_ = 0;
};

} 

#endif 
