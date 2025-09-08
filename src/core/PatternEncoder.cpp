#include "pointcloud_compressor/core/PatternEncoder.hpp"
#include <fstream>
#include <algorithm>
#include <iostream>

namespace pointcloud_compressor {

PatternEncoder::PatternEncoder() {}

PatternEncoder::~PatternEncoder() {}

bool PatternEncoder::encodePatterns(const std::vector<uint16_t>& indices, 
                                  const std::string& output_filename) {
    return writeIndices16bit(indices, output_filename);
}

bool PatternEncoder::encodePatterns8bit(const std::vector<uint16_t>& indices, 
                                       const std::string& output_filename) {
    if (!canUse8bitIndices(indices)) {
        return false;
    }
    return writeIndices8bit(indices, output_filename);
}

bool PatternEncoder::decodePatterns(const std::string& input_filename, 
                                   std::vector<uint16_t>& indices) {
    return readIndices(input_filename, indices);
}

bool PatternEncoder::encodePatternsAuto(const std::vector<uint16_t>& indices,
                                       const std::string& output_filename) {
    // Automatically choose 8-bit or 16-bit based on max value
    if (canUse8bitIndices(indices)) {
        return writeIndices8bit(indices, output_filename);
    } else {
        return writeIndices16bit(indices, output_filename);
    }
}

bool PatternEncoder::canUse8bitIndices(const std::vector<uint16_t>& indices) {
    if (indices.empty()) {
        return true;  // Empty indices can use 8-bit
    }
    auto max_it = std::max_element(indices.begin(), indices.end());
    return *max_it < 256;
}

bool PatternEncoder::writeIndices16bit(const std::vector<uint16_t>& indices, 
                                      const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    // Write header
    uint32_t num_indices = static_cast<uint32_t>(indices.size());
    uint8_t index_size = 16;  // 16-bit indices
    
    file.write(reinterpret_cast<const char*>(&num_indices), sizeof(num_indices));
    file.write(reinterpret_cast<const char*>(&index_size), sizeof(index_size));
    
    // Write indices
    for (uint16_t index : indices) {
        file.write(reinterpret_cast<const char*>(&index), sizeof(index));
    }
    
    return file.good();
}

bool PatternEncoder::writeIndices8bit(const std::vector<uint16_t>& indices, 
                                     const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    // Write header
    uint32_t num_indices = static_cast<uint32_t>(indices.size());
    uint8_t index_size = 8;  // 8-bit indices
    
    file.write(reinterpret_cast<const char*>(&num_indices), sizeof(num_indices));
    file.write(reinterpret_cast<const char*>(&index_size), sizeof(index_size));
    
    // Write indices as 8-bit
    for (uint16_t index : indices) {
        uint8_t index8 = static_cast<uint8_t>(index);
        file.write(reinterpret_cast<const char*>(&index8), sizeof(index8));
    }
    
    return file.good();
}

bool PatternEncoder::readIndices(const std::string& filename, 
                                std::vector<uint16_t>& indices) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    indices.clear();
    
    // Read header
    uint32_t num_indices;
    uint8_t index_size;
    
    file.read(reinterpret_cast<char*>(&num_indices), sizeof(num_indices));
    file.read(reinterpret_cast<char*>(&index_size), sizeof(index_size));
    
    if (!file.good()) {
        return false;
    }
    
    indices.reserve(num_indices);
    
    // Read indices based on size
    if (index_size == 8) {
        for (uint32_t i = 0; i < num_indices; ++i) {
            uint8_t index8;
            file.read(reinterpret_cast<char*>(&index8), sizeof(index8));
            indices.push_back(static_cast<uint16_t>(index8));
        }
    } else if (index_size == 16) {
        for (uint32_t i = 0; i < num_indices; ++i) {
            uint16_t index16;
            file.read(reinterpret_cast<char*>(&index16), sizeof(index16));
            indices.push_back(index16);
        }
    } else {
        return false;  // Unsupported index size
    }
    
    return file.good();
}

} // namespace pointcloud_compressor