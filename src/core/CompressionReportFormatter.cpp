// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_compressor/core/CompressionReportFormatter.hpp"

#include <iomanip>
#include <sstream>

namespace pointcloud_compressor {

std::string formatCompressionSummary(const PCCCompressionReport& report) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "Compression summary:\n";
    oss << "  Compression ratio: " << report.statistics.compression_ratio << "\n";
    oss << "  Block count      : " << report.indices.total_blocks << "\n";
    oss << "  Dictionary size  : " << report.dictionary.num_patterns
        << " patterns (" << report.dictionary.pattern_size_bytes << " bytes each)\n";
    oss << "  Index bit width  : " << static_cast<int>(report.indices.index_bit_size) << " bits";
    return oss.str();
}

}  // namespace pointcloud_compressor
