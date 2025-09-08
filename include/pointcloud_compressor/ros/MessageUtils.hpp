#ifndef POINTCLOUD_COMPRESSOR_MESSAGE_UTILS_HPP
#define POINTCLOUD_COMPRESSOR_MESSAGE_UTILS_HPP

#include <vector>
#include <cstdint>
#include "pointcloud_compressor/msg/pattern_dictionary.hpp"

namespace pointcloud_compressor {
namespace ros {

/**
 * @brief Utility functions for handling ROS messages with dynamic index encoding
 */
class MessageUtils {
public:
    /**
     * @brief Extract block indices from PatternDictionary message
     * 
     * Handles both 8-bit and 16-bit encoded indices based on index_bit_size field
     * 
     * @param msg PatternDictionary message
     * @return vector of 16-bit indices
     */
    static std::vector<uint16_t> extractBlockIndices(
        const pointcloud_compressor::msg::PatternDictionary& msg) 
    {
        std::vector<uint16_t> indices;
        
        if (msg.index_bit_size == 8) {
            // 8-bit encoding: each byte is one index
            indices.reserve(msg.block_indices_data.size());
            for (uint8_t byte : msg.block_indices_data) {
                indices.push_back(static_cast<uint16_t>(byte));
            }
        } else if (msg.index_bit_size == 16) {
            // 16-bit encoding: two bytes form one index (little-endian)
            size_t num_indices = msg.block_indices_data.size() / 2;
            indices.reserve(num_indices);
            
            for (size_t i = 0; i < msg.block_indices_data.size(); i += 2) {
                uint16_t idx = static_cast<uint16_t>(msg.block_indices_data[i]) |
                              (static_cast<uint16_t>(msg.block_indices_data[i + 1]) << 8);
                indices.push_back(idx);
            }
        }
        
        return indices;
    }
    
    /**
     * @brief Pack indices into byte array for PatternDictionary message
     * 
     * Automatically determines whether to use 8-bit or 16-bit encoding
     * based on maximum index value
     * 
     * @param indices Vector of indices to pack
     * @param[out] index_bit_size Will be set to 8 or 16
     * @param[out] packed_data Packed byte array
     */
    static void packBlockIndices(
        const std::vector<uint16_t>& indices,
        uint8_t& index_bit_size,
        std::vector<uint8_t>& packed_data)
    {
        packed_data.clear();
        
        // Find maximum index
        uint16_t max_index = 0;
        if (!indices.empty()) {
            max_index = *std::max_element(indices.begin(), indices.end());
        }
        
        if (max_index < 256) {
            // Use 8-bit encoding
            index_bit_size = 8;
            packed_data.reserve(indices.size());
            
            for (uint16_t idx : indices) {
                packed_data.push_back(static_cast<uint8_t>(idx));
            }
        } else {
            // Use 16-bit encoding (little-endian)
            index_bit_size = 16;
            packed_data.reserve(indices.size() * 2);
            
            for (uint16_t idx : indices) {
                packed_data.push_back(static_cast<uint8_t>(idx & 0xFF));         // Low byte
                packed_data.push_back(static_cast<uint8_t>((idx >> 8) & 0xFF)); // High byte
            }
        }
    }
    
    /**
     * @brief Calculate the space savings from using optimal index encoding
     * 
     * @param indices Vector of indices
     * @return Bytes saved by using 8-bit encoding when possible
     */
    static size_t calculateIndexEncodingSavings(const std::vector<uint16_t>& indices)
    {
        if (indices.empty()) {
            return 0;
        }
        
        uint16_t max_index = *std::max_element(indices.begin(), indices.end());
        
        if (max_index < 256) {
            // Using 8-bit saves 1 byte per index compared to 16-bit
            return indices.size();
        }
        
        return 0; // No savings if 16-bit encoding is required
    }
};

} // namespace ros
} // namespace pointcloud_compressor

#endif // POINTCLOUD_COMPRESSOR_MESSAGE_UTILS_HPP