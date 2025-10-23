// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_compressor/core/PatternDictionaryBuilder.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <chrono>

namespace pointcloud_compressor {

PatternDictionaryBuilder::PatternDictionaryBuilder() {}

PatternDictionaryBuilder::~PatternDictionaryBuilder() {}

bool PatternDictionaryBuilder::buildDictionary(const std::vector<std::vector<uint8_t>>& patterns) {
    auto t0 = std::chrono::high_resolution_clock::now();
    unique_patterns_.clear();
    pattern_indices_.clear();
    pattern_to_index_.clear();
    max_index_ = 0;
    
    if (patterns.empty()) {
        return false;
    }
    
    uint64_t next_index = 0;
    
    for (const auto& pattern : patterns) {
        auto it = pattern_to_index_.find(pattern);
        if (it == pattern_to_index_.end()) {
            // New unique pattern
            pattern_to_index_[pattern] = next_index;
            unique_patterns_.push_back(pattern);
            pattern_indices_.push_back(next_index);
            max_index_ = next_index;
            next_index++;
        } else {
            // Existing pattern
            pattern_indices_.push_back(it->second);
        }
    }
    
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    std::cout << "[PROFILE][DictionaryBuilder] patterns=" << patterns.size()
              << ", unique=" << unique_patterns_.size()
              << ", time=" << ms << " ms" << std::endl;
    return !unique_patterns_.empty();
}

const std::vector<std::vector<uint8_t>>& PatternDictionaryBuilder::getUniquePatterns() const {
    return unique_patterns_;
}

const std::vector<uint64_t>& PatternDictionaryBuilder::getPatternIndices() const {
    return pattern_indices_;
}

int PatternDictionaryBuilder::getRequiredIndexBitSize() const {
    if (max_index_ <= std::numeric_limits<uint8_t>::max()) {
        return 8;
    } else if (max_index_ <= std::numeric_limits<uint16_t>::max()) {
        return 16;
    } else if (max_index_ <= std::numeric_limits<uint32_t>::max()) {
        return 32;
    } else {
        return 64;
    }
}

bool PatternDictionaryBuilder::saveDictionary(const std::string& filename, int index_bit_size) const {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    // Write number of unique patterns
    uint32_t num_patterns = static_cast<uint32_t>(unique_patterns_.size());
    file.write(reinterpret_cast<const char*>(&num_patterns), sizeof(num_patterns));
    
    // Write patterns
    for (const auto& pattern : unique_patterns_) {
        uint32_t pattern_size = static_cast<uint32_t>(pattern.size());
        file.write(reinterpret_cast<const char*>(&pattern_size), sizeof(pattern_size));
        file.write(reinterpret_cast<const char*>(pattern.data()), pattern_size);
    }
    
    return file.good();
}

bool PatternDictionaryBuilder::loadDictionary(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    unique_patterns_.clear();
    pattern_to_index_.clear();
    
    // Read number of patterns
    uint32_t num_patterns;
    file.read(reinterpret_cast<char*>(&num_patterns), sizeof(num_patterns));
    
    if (!file.good()) {
        return false;
    }
    
    // Read patterns
    for (uint32_t i = 0; i < num_patterns; ++i) {
        uint32_t pattern_size;
        file.read(reinterpret_cast<char*>(&pattern_size), sizeof(pattern_size));
        
        std::vector<uint8_t> pattern(pattern_size);
        file.read(reinterpret_cast<char*>(pattern.data()), pattern_size);
        
        if (!file.good()) {
            return false;
        }
        
        unique_patterns_.push_back(pattern);
        pattern_to_index_[pattern] = static_cast<uint64_t>(i);
        max_index_ = std::max(max_index_, static_cast<uint64_t>(i));
    }
    
    return true;
}

size_t PatternDictionaryBuilder::getUniquePatternCount() const {
    return unique_patterns_.size();
}

float PatternDictionaryBuilder::getCompressionRatio() const {
    if (pattern_indices_.empty() || unique_patterns_.empty()) {
        return 1.0f;
    }
    
    // Calculate original size - sum of all patterns if stored without dictionary
    size_t original_bits = 0;
    size_t avg_pattern_size = 0;
    
    // Calculate average pattern size
    for (const auto& pattern : unique_patterns_) {
        avg_pattern_size += pattern.size();
    }
    if (!unique_patterns_.empty()) {
        avg_pattern_size /= unique_patterns_.size();
    }
    
    // Original size = number of patterns * average pattern size in bits
    original_bits = pattern_indices_.size() * avg_pattern_size * 8;
    
    // Calculate compressed size (dictionary + indices)
    size_t dictionary_bits = 0;
    for (const auto& pattern : unique_patterns_) {
        dictionary_bits += pattern.size() * 8;
    }
    
    size_t index_bits = pattern_indices_.size() * 16;  // 16 bits per index
    size_t compressed_bits = dictionary_bits + index_bits;
    
    if (original_bits == 0) {
        return 1.0f;
    }
    
    return static_cast<float>(compressed_bits) / static_cast<float>(original_bits);
}

void PatternDictionaryBuilder::setDictionary(const std::vector<std::vector<uint8_t>>& patterns) {
    unique_patterns_ = patterns;
    pattern_to_index_.clear();
    pattern_indices_.clear();
    max_index_ = 0;

    uint64_t index = 0;
    for (const auto& pattern : unique_patterns_) {
        pattern_to_index_[pattern] = index;
        max_index_ = index;
        ++index;
    }
}

} // namespace pointcloud_compressor
