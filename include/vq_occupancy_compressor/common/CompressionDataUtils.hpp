// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_COMMON_COMPRESSION_DATA_UTILS_HPP
#define VQ_OCCUPANCY_COMPRESSOR_COMMON_COMPRESSION_DATA_UTILS_HPP

#include <cstdint>
#include <vector>

namespace vq_occupancy_compressor::common {

uint8_t bitWidthFromMaxIndex(uint64_t max_index);
std::vector<uint32_t> convertBlockIndicesToU32(const std::vector<uint64_t>& indices);

}  // namespace vq_occupancy_compressor::common

#endif  // VQ_OCCUPANCY_COMPRESSOR_COMMON_COMPRESSION_DATA_UTILS_HPP
