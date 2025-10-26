// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_COMMON_RUNTIME_HELPERS_HPP
#define POINTCLOUD_COMPRESSOR_COMMON_RUNTIME_HELPERS_HPP

#include <cstdint>
#include <vector>

#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/bridge/Bridge.hpp"

namespace pointcloud_compressor::common {

uint8_t bitWidthFromMaxIndex(uint64_t max_index);
std::vector<uint32_t> convertBlockIndicesToU32(const std::vector<uint64_t>& indices);

}  // namespace pointcloud_compressor::common

#endif  // POINTCLOUD_COMPRESSOR_COMMON_RUNTIME_HELPERS_HPP
