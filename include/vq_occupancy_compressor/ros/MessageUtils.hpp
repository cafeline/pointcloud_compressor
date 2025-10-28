// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_MESSAGE_UTILS_HPP
#define VQ_OCCUPANCY_COMPRESSOR_MESSAGE_UTILS_HPP

#include <vector>
#include <cstdint>
#include <algorithm>
#include "vq_occupancy_compressor/msg/pattern_dictionary.hpp"

namespace vq_occupancy_compressor {
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
        const vq_occupancy_compressor::msg::PatternDictionary& msg) 
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

    /**
     * 追加: 32/64bitエンコード/デコード（上位互換）
     */
    static std::vector<uint64_t> extractBlockIndices64(
        const vq_occupancy_compressor::msg::PatternDictionary& msg)
    {
        std::vector<uint64_t> indices;
        const auto& data = msg.block_indices_data;
        switch (msg.index_bit_size) {
            case 8: {
                indices.reserve(data.size());
                for (uint8_t b : data) indices.push_back(static_cast<uint64_t>(b));
                break;
            }
            case 16: {
                size_t n = data.size() / 2;
                indices.reserve(n);
                for (size_t i = 0; i + 1 < data.size(); i += 2) {
                    uint16_t v = static_cast<uint16_t>(data[i]) |
                                 (static_cast<uint16_t>(data[i+1]) << 8);
                    indices.push_back(static_cast<uint64_t>(v));
                }
                break;
            }
            case 32: {
                size_t n = data.size() / 4;
                indices.reserve(n);
                for (size_t i = 0; i + 3 < data.size(); i += 4) {
                    uint32_t v = static_cast<uint32_t>(data[i]) |
                                 (static_cast<uint32_t>(data[i+1]) << 8) |
                                 (static_cast<uint32_t>(data[i+2]) << 16) |
                                 (static_cast<uint32_t>(data[i+3]) << 24);
                    indices.push_back(static_cast<uint64_t>(v));
                }
                break;
            }
            case 64: {
                size_t n = data.size() / 8;
                indices.reserve(n);
                for (size_t i = 0; i + 7 < data.size(); i += 8) {
                    uint64_t v =  static_cast<uint64_t>(data[i]) |
                                 (static_cast<uint64_t>(data[i+1]) << 8) |
                                 (static_cast<uint64_t>(data[i+2]) << 16) |
                                 (static_cast<uint64_t>(data[i+3]) << 24) |
                                 (static_cast<uint64_t>(data[i+4]) << 32) |
                                 (static_cast<uint64_t>(data[i+5]) << 40) |
                                 (static_cast<uint64_t>(data[i+6]) << 48) |
                                 (static_cast<uint64_t>(data[i+7]) << 56);
                    indices.push_back(v);
                }
                break;
            }
            default:
                // 未対応サイズ
                break;
        }
        return indices;
    }

    static void packBlockIndices(
        const std::vector<uint64_t>& indices,
        uint8_t& index_bit_size,
        std::vector<uint8_t>& packed_data)
    {
        packed_data.clear();
        uint64_t max_index = 0;
        if (!indices.empty()) {
            max_index = *std::max_element(indices.begin(), indices.end());
        }

        if (max_index <= 0xFFull) {
            index_bit_size = 8;
            packed_data.reserve(indices.size());
            for (uint64_t v : indices) {
                packed_data.push_back(static_cast<uint8_t>(v & 0xFF));
            }
        } else if (max_index <= 0xFFFFull) {
            index_bit_size = 16;
            packed_data.reserve(indices.size() * 2);
            for (uint64_t v : indices) {
                uint16_t w = static_cast<uint16_t>(v);
                packed_data.push_back(static_cast<uint8_t>(w & 0xFF));
                packed_data.push_back(static_cast<uint8_t>((w >> 8) & 0xFF));
            }
        } else if (max_index <= 0xFFFF'FFFFull) {
            index_bit_size = 32;
            packed_data.reserve(indices.size() * 4);
            for (uint64_t v : indices) {
                uint32_t w = static_cast<uint32_t>(v);
                packed_data.push_back(static_cast<uint8_t>(w & 0xFF));
                packed_data.push_back(static_cast<uint8_t>((w >> 8) & 0xFF));
                packed_data.push_back(static_cast<uint8_t>((w >> 16) & 0xFF));
                packed_data.push_back(static_cast<uint8_t>((w >> 24) & 0xFF));
            }
        } else {
            index_bit_size = 64;
            packed_data.reserve(indices.size() * 8);
            for (uint64_t w : indices) {
                packed_data.push_back(static_cast<uint8_t>(w & 0xFF));
                packed_data.push_back(static_cast<uint8_t>((w >> 8) & 0xFF));
                packed_data.push_back(static_cast<uint8_t>((w >> 16) & 0xFF));
                packed_data.push_back(static_cast<uint8_t>((w >> 24) & 0xFF));
                packed_data.push_back(static_cast<uint8_t>((w >> 32) & 0xFF));
                packed_data.push_back(static_cast<uint8_t>((w >> 40) & 0xFF));
                packed_data.push_back(static_cast<uint8_t>((w >> 48) & 0xFF));
                packed_data.push_back(static_cast<uint8_t>((w >> 56) & 0xFF));
            }
        }
    }
};

} // namespace ros
} // namespace vq_occupancy_compressor

#endif // VQ_OCCUPANCY_COMPRESSOR_MESSAGE_UTILS_HPP
