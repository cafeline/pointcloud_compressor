// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_compressor/core/BlockSizeReportFormatter.hpp"

#include <iomanip>
#include <sstream>

namespace pointcloud_compressor {

std::string formatBlockSizeSummary(const BlockSizeOptimizationResult& result,
                                   bool verbose) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);

    if (result.optimal_block_size < 0) {
        oss << "Block size optimization failed.";
        return oss.str();
    }

    oss << "Optimal block size: " << result.optimal_block_size << '\n';
    oss << "Best compression ratio: " << result.best_compression_ratio << '\n';

    if (verbose && !result.tested_results.empty()) {
        oss << "Tested block sizes:\n";
        for (const auto& [block_size, ratio] : result.tested_results) {
            oss << "  block_size=" << block_size << " ratio=" << ratio << '\n';
        }
    }

    return oss.str();
}

}  // namespace pointcloud_compressor
