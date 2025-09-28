#include "pointcloud_compressor/core/PatternEncoder.hpp"
#include <fstream>
#include <algorithm>
#include <iostream>
#include <limits>
#include <chrono>

namespace pointcloud_compressor {

PatternEncoder::PatternEncoder() {}

PatternEncoder::~PatternEncoder() {}

bool PatternEncoder::encodePatterns(const std::vector<uint16_t>& indices, 
                                  const std::string& output_filename) {
    // Convert to uint64_t vector for compatibility
    std::vector<uint64_t> indices64(indices.begin(), indices.end());
    return encodePatternsWithBitSize(indices64, output_filename, 16);
}

bool PatternEncoder::encodePatterns8bit(const std::vector<uint16_t>& indices, 
                                       const std::string& output_filename) {
    if (!canUse8bitIndices(indices)) {
        return false;
    }
    std::vector<uint64_t> indices64(indices.begin(), indices.end());
    return encodePatternsWithBitSize(indices64, output_filename, 8);
}

bool PatternEncoder::decodePatterns(const std::string& input_filename, 
                                   std::vector<uint64_t>& indices) {
    auto t0 = std::chrono::high_resolution_clock::now();
    bool ok = readIndices(input_filename, indices);
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    std::cout << "[PROFILE][PatternEncoder] decode indices=" << indices.size()
              << ", time=" << ms << " ms" << std::endl;
    return ok;
}

bool PatternEncoder::encodePatternsAuto(const std::vector<uint64_t>& indices,
                                       const std::string& output_filename) {
    if (indices.empty()) {
        return writeIndices8bit(indices, output_filename);
    }
    
    // Find maximum index value
    uint64_t max_index = *std::max_element(indices.begin(), indices.end());
    int bit_size = getRequiredBitSize(max_index);
    
    return encodePatternsWithBitSize(indices, output_filename, bit_size);
}

bool PatternEncoder::encodePatternsWithBitSize(const std::vector<uint64_t>& indices,
                                              const std::string& output_filename,
                                              int bit_size) {
    auto t0 = std::chrono::high_resolution_clock::now();
    bool ok = false;
    switch (bit_size) {
        case 8:
            ok = writeIndices8bit(indices, output_filename);
            break;
        case 16:
            ok = writeIndices16bit(indices, output_filename);
            break;
        case 32:
            ok = writeIndices32bit(indices, output_filename);
            break;
        case 64:
            ok = writeIndices64bit(indices, output_filename);
            break;
        default:
            std::cerr << "Invalid bit size: " << bit_size << std::endl;
            return false;
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    std::cout << "[PROFILE][PatternEncoder] encode " << bit_size << "-bit indices=" << indices.size()
              << ", time=" << ms << " ms" << std::endl;
    return ok;
}

int PatternEncoder::getRequiredBitSize(uint64_t max_value) {
    if (max_value <= std::numeric_limits<uint8_t>::max()) {
        return 8;
    } else if (max_value <= std::numeric_limits<uint16_t>::max()) {
        return 16;
    } else if (max_value <= std::numeric_limits<uint32_t>::max()) {
        return 32;
    } else {
        return 64;
    }
}

bool PatternEncoder::canUse8bitIndices(const std::vector<uint16_t>& indices) {
    if (indices.empty()) {
        return true;  // Empty indices can use 8-bit
    }
    auto max_it = std::max_element(indices.begin(), indices.end());
    return *max_it < 256;
}

bool PatternEncoder::writeIndices16bit(const std::vector<uint64_t>& indices, 
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
    for (uint64_t index : indices) {
        uint16_t index16 = static_cast<uint16_t>(index);
        file.write(reinterpret_cast<const char*>(&index16), sizeof(index16));
    }
    
    return file.good();
}

bool PatternEncoder::writeIndices8bit(const std::vector<uint64_t>& indices, 
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
    for (uint64_t index : indices) {
        uint8_t index8 = static_cast<uint8_t>(index);
        file.write(reinterpret_cast<const char*>(&index8), sizeof(index8));
    }
    
    return file.good();
}

bool PatternEncoder::writeIndices32bit(const std::vector<uint64_t>& indices, 
                                      const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    // Write header
    uint32_t num_indices = static_cast<uint32_t>(indices.size());
    uint8_t index_size = 32;  // 32-bit indices
    
    file.write(reinterpret_cast<const char*>(&num_indices), sizeof(num_indices));
    file.write(reinterpret_cast<const char*>(&index_size), sizeof(index_size));
    
    // Write indices
    for (uint64_t index : indices) {
        uint32_t index32 = static_cast<uint32_t>(index);
        file.write(reinterpret_cast<const char*>(&index32), sizeof(index32));
    }
    
    return file.good();
}

bool PatternEncoder::writeIndices64bit(const std::vector<uint64_t>& indices, 
                                      const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    // Write header
    uint32_t num_indices = static_cast<uint32_t>(indices.size());
    uint8_t index_size = 64;  // 64-bit indices
    
    file.write(reinterpret_cast<const char*>(&num_indices), sizeof(num_indices));
    file.write(reinterpret_cast<const char*>(&index_size), sizeof(index_size));
    
    // Write indices
    for (uint64_t index : indices) {
        file.write(reinterpret_cast<const char*>(&index), sizeof(index));
    }
    
    return file.good();
}

bool PatternEncoder::readIndices(const std::string& filename, 
                                std::vector<uint64_t>& indices) {
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
            indices.push_back(static_cast<uint64_t>(index8));
        }
    } else if (index_size == 16) {
        for (uint32_t i = 0; i < num_indices; ++i) {
            uint16_t index16;
            file.read(reinterpret_cast<char*>(&index16), sizeof(index16));
            indices.push_back(static_cast<uint64_t>(index16));
        }
    } else if (index_size == 32) {
        for (uint32_t i = 0; i < num_indices; ++i) {
            uint32_t index32;
            file.read(reinterpret_cast<char*>(&index32), sizeof(index32));
            indices.push_back(static_cast<uint64_t>(index32));
        }
    } else if (index_size == 64) {
        for (uint32_t i = 0; i < num_indices; ++i) {
            uint64_t index64;
            file.read(reinterpret_cast<char*>(&index64), sizeof(index64));
            indices.push_back(index64);
        }
    } else {
        std::cerr << "Unsupported index size: " << static_cast<int>(index_size) << std::endl;
        return false;  // Unsupported index size
    }
    
    return file.good();
}

} // namespace pointcloud_compressor
