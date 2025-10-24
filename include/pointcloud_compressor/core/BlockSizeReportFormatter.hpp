// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_CORE_BLOCK_SIZE_REPORT_FORMATTER_HPP
#define POINTCLOUD_COMPRESSOR_CORE_BLOCK_SIZE_REPORT_FORMATTER_HPP

#include <string>

#include "pointcloud_compressor/core/PointCloudCompressor.hpp"

namespace pointcloud_compressor {

std::string formatBlockSizeSummary(const BlockSizeOptimizationResult& result,
                                   bool verbose);

}  // namespace pointcloud_compressor

#endif  // POINTCLOUD_COMPRESSOR_CORE_BLOCK_SIZE_REPORT_FORMATTER_HPP
