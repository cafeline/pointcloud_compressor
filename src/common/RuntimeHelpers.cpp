// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_compressor/common/RuntimeHelpers.hpp"

#include <algorithm>

namespace pointcloud_compressor::common {

uint8_t bitWidthFromMaxIndex(uint64_t max_index) {
    if (max_index == 0) {
        return 1;
    }

    uint8_t bits = 0;
    while (max_index > 0) {
        ++bits;
        max_index >>= 1;
    }

    return std::min<uint8_t>(bits, static_cast<uint8_t>(32));
}

std::vector<uint32_t> convertBlockIndicesToU32(const std::vector<uint64_t>& indices) {
    std::vector<uint32_t> converted;
    converted.reserve(indices.size());
    for (uint64_t value : indices) {
        converted.push_back(static_cast<uint32_t>(value));
    }
    return converted;
}

}  // namespace pointcloud_compressor::common
