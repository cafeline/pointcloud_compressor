// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_PATTERN_ENCODER_HPP
#define POINTCLOUD_COMPRESSOR_PATTERN_ENCODER_HPP

#include <vector>
#include <string>
#include <cstdint>
#include <limits>

namespace pointcloud_compressor {

class PatternEncoder {
public:
    PatternEncoder();
    ~PatternEncoder();
    
    // Encode patterns with specified bit size (8, 16, 32, or 64)
    bool encodePatternsWithBitSize(const std::vector<uint64_t>& indices,
                                   const std::string& output_filename,
                                   int bit_size);
    
    // Legacy methods for compatibility (will use dynamic sizing internally)
    bool encodePatterns(const std::vector<uint16_t>& indices, 
                       const std::string& output_filename);
    
    bool encodePatterns8bit(const std::vector<uint16_t>& indices, 
                           const std::string& output_filename);
    
    bool encodePatternsAuto(const std::vector<uint64_t>& indices,
                           const std::string& output_filename);
    
    // Decode patterns (automatically detects bit size)
    bool decodePatterns(const std::string& input_filename, 
                       std::vector<uint64_t>& indices);
    
    // Utility functions
    int getRequiredBitSize(uint64_t max_value);
    bool canUse8bitIndices(const std::vector<uint16_t>& indices);
    
private:
    bool writeIndices8bit(const std::vector<uint64_t>& indices, 
                         const std::string& filename);
    bool writeIndices16bit(const std::vector<uint64_t>& indices, 
                          const std::string& filename);
    bool writeIndices32bit(const std::vector<uint64_t>& indices, 
                          const std::string& filename);
    bool writeIndices64bit(const std::vector<uint64_t>& indices, 
                          const std::string& filename);
    bool readIndices(const std::string& filename, 
                    std::vector<uint64_t>& indices);
};

} // namespace pointcloud_compressor

#endif // POINTCLOUD_COMPRESSOR_PATTERN_ENCODER_HPP