// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_CORE_COMPRESSION_REPORT_FORMATTER_HPP
#define POINTCLOUD_COMPRESSOR_CORE_COMPRESSION_REPORT_FORMATTER_HPP

#include <string>

#include "pointcloud_compressor/runtime/RuntimeAPI.hpp"

namespace pointcloud_compressor {

std::string formatCompressionSummary(const PCCCompressionReport& report);

}  // namespace pointcloud_compressor

#endif  // POINTCLOUD_COMPRESSOR_CORE_COMPRESSION_REPORT_FORMATTER_HPP
