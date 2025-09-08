#ifndef POINTCLOUD_COMPRESSOR_PATTERN_ENCODER_HPP
#define POINTCLOUD_COMPRESSOR_PATTERN_ENCODER_HPP

#include <vector>
#include <string>
#include <cstdint>

namespace pointcloud_compressor {

class PatternEncoder {
public:
    PatternEncoder();
    ~PatternEncoder();
    
    // Encode patterns using 16-bit indices
    bool encodePatterns(const std::vector<uint16_t>& indices, 
                       const std::string& output_filename);
    
    // Encode patterns using 8-bit indices (if possible)
    bool encodePatterns8bit(const std::vector<uint16_t>& indices, 
                           const std::string& output_filename);
    
    // Automatically choose 8-bit or 16-bit encoding based on max index value
    bool encodePatternsAuto(const std::vector<uint16_t>& indices,
                           const std::string& output_filename);
    
    // Decode patterns
    bool decodePatterns(const std::string& input_filename, 
                       std::vector<uint16_t>& indices);
    
    // Check if 8-bit encoding is possible
    bool canUse8bitIndices(const std::vector<uint16_t>& indices);
    
private:
    bool writeIndices16bit(const std::vector<uint16_t>& indices, 
                          const std::string& filename);
    bool writeIndices8bit(const std::vector<uint16_t>& indices, 
                         const std::string& filename);
    bool readIndices(const std::string& filename, 
                    std::vector<uint16_t>& indices);
};

} // namespace pointcloud_compressor

#endif // POINTCLOUD_COMPRESSOR_PATTERN_ENCODER_HPP