#ifndef POINTCLOUD_COMPRESSOR_PATTERN_DICTIONARY_BUILDER_HPP
#define POINTCLOUD_COMPRESSOR_PATTERN_DICTIONARY_BUILDER_HPP

#include <vector>
#include <map>
#include <string>
#include <cstdint>

namespace pointcloud_compressor {

class PatternDictionaryBuilder {
public:
    PatternDictionaryBuilder();
    ~PatternDictionaryBuilder();
    
    // Build dictionary from patterns
    bool buildDictionary(const std::vector<std::vector<uint8_t>>& patterns);
    
    // Get unique patterns
    const std::vector<std::vector<uint8_t>>& getUniquePatterns() const;
    
    // Get pattern indices
    const std::vector<uint16_t>& getPatternIndices() const;
    
    // Save/load dictionary
    bool saveDictionary(const std::string& filename) const;
    bool loadDictionary(const std::string& filename);
    
    // Statistics
    size_t getUniquePatternCount() const;
    float getCompressionRatio() const;
    
private:
    std::vector<std::vector<uint8_t>> unique_patterns_;
    std::vector<uint16_t> pattern_indices_;
    std::map<std::vector<uint8_t>, uint16_t> pattern_to_index_;
};

} // namespace pointcloud_compressor

#endif // POINTCLOUD_COMPRESSOR_PATTERN_DICTIONARY_BUILDER_HPP