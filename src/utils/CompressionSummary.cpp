// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "vq_occupancy_compressor/utils/CompressionSummary.hpp"

#include <algorithm>
#include <cmath>

#include "vq_occupancy_compressor/model/VoxelGrid.hpp"

namespace vq_occupancy_compressor::utils {

namespace {

inline std::size_t computeOneBitBytes(std::size_t voxels) {
    return voxels > 0 ? (voxels + 7) / 8 : 0;
}

CompressionSummaryMetrics computeFromCommon(std::size_t total_voxels,
                                            double env_x,
                                            double env_y,
                                            double env_z,
                                            std::size_t dictionary_entries,
                                            std::size_t dictionary_bytes,
                                            std::size_t index_entries,
                                            std::size_t index_bytes,
                                            std::size_t compressed_bytes) {
    CompressionSummaryMetrics metrics{};
    metrics.env_x = env_x;
    metrics.env_y = env_y;
    metrics.env_z = env_z;

    metrics.total_voxels = total_voxels;
    metrics.bytes_one_byte = total_voxels;
    metrics.bytes_one_bit = computeOneBitBytes(total_voxels);

    metrics.dictionary_entries = dictionary_entries;
    metrics.dictionary_bytes = dictionary_bytes;
    metrics.index_entries = index_entries;
    metrics.index_bytes = index_bytes;
    metrics.compressed_bytes = compressed_bytes;

    metrics.ratio_vs_one_byte = metrics.bytes_one_byte > 0
        ? static_cast<double>(metrics.compressed_bytes) /
              static_cast<double>(metrics.bytes_one_byte)
        : 0.0;
    metrics.ratio_vs_one_bit = metrics.bytes_one_bit > 0
        ? static_cast<double>(metrics.compressed_bytes) /
              static_cast<double>(metrics.bytes_one_bit)
        : 0.0;
    return metrics;
}

inline double safeVoxelSize(float value, double fallback) {
    if (value > 0.0f) {
        return static_cast<double>(value);
    }
    return fallback;
}

}  // namespace

CompressionSummaryMetrics computeSummaryMetrics(const CompressionResult& result) {
    const VoxelGrid& grid = result.voxel_grid;
    const VoxelCoord dims = grid.getDimensions();
    const double voxel_size = safeVoxelSize(result.voxel_size, 0.0);

    std::size_t coverage_x = static_cast<std::size_t>(result.blocks_count.x) * static_cast<std::size_t>(result.block_size);
    std::size_t coverage_y = static_cast<std::size_t>(result.blocks_count.y) * static_cast<std::size_t>(result.block_size);
    std::size_t coverage_z = static_cast<std::size_t>(result.blocks_count.z) * static_cast<std::size_t>(result.block_size);

    coverage_x = std::max<std::size_t>(coverage_x, static_cast<std::size_t>(dims.x));
    coverage_y = std::max<std::size_t>(coverage_y, static_cast<std::size_t>(dims.y));
    coverage_z = std::max<std::size_t>(coverage_z, static_cast<std::size_t>(dims.z));

    const std::size_t total_voxels = coverage_x * coverage_y * coverage_z;
    const double env_x = static_cast<double>(coverage_x) * voxel_size;
    const double env_y = static_cast<double>(coverage_y) * voxel_size;
    const double env_z = static_cast<double>(coverage_z) * voxel_size;

    std::size_t dictionary_bytes = 0;
    for (const auto& pattern : result.pattern_dictionary) {
        dictionary_bytes += pattern.size();
    }
    const std::size_t dictionary_entries = result.pattern_dictionary.size();

    const std::size_t index_entries = result.block_indices.size();
    const std::size_t index_entry_bytes = (result.index_bit_size + 7) / 8;
    const std::size_t index_bytes = index_entries * index_entry_bytes;

    std::size_t compressed_bytes = result.compressed_size;
    if (compressed_bytes == 0) {
        compressed_bytes = dictionary_bytes + index_bytes;
    }

    return computeFromCommon(total_voxels,
                             env_x,
                             env_y,
                             env_z,
                             dictionary_entries,
                             dictionary_bytes,
                             index_entries,
                             index_bytes,
                             compressed_bytes);
}

CompressionSummaryMetrics computeSummaryMetrics(const PCCCompressionReport& report) {
    const double voxel_size = safeVoxelSize(static_cast<float>(report.grid.voxel_size), 0.0);

    std::size_t block_size = 0;
    if (report.dictionary.pattern_size_bytes > 0) {
        const double pattern_bits = static_cast<double>(report.dictionary.pattern_size_bytes) * 8.0;
        block_size = static_cast<std::size_t>(std::llround(std::cbrt(pattern_bits)));
    }
    if (block_size == 0) {
        block_size = 1;
    }

    auto coverage_from_blocks = [&](int index) {
        return static_cast<std::size_t>(report.grid.blocks_per_axis[index]) * block_size;
    };

    std::size_t coverage_x = coverage_from_blocks(0);
    std::size_t coverage_y = coverage_from_blocks(1);
    std::size_t coverage_z = coverage_from_blocks(2);

    if (voxel_size > 0.0) {
        coverage_x = std::max<std::size_t>(coverage_x,
            static_cast<std::size_t>(std::llround(report.grid.dimensions[0] / voxel_size)));
        coverage_y = std::max<std::size_t>(coverage_y,
            static_cast<std::size_t>(std::llround(report.grid.dimensions[1] / voxel_size)));
        coverage_z = std::max<std::size_t>(coverage_z,
            static_cast<std::size_t>(std::llround(report.grid.dimensions[2] / voxel_size)));
    }

    if (coverage_x == 0) coverage_x = 1;
    if (coverage_y == 0) coverage_y = 1;
    if (coverage_z == 0) coverage_z = 1;

    const std::size_t total_voxels = coverage_x * coverage_y * coverage_z;
    const double env_x = voxel_size > 0.0 ? static_cast<double>(coverage_x) * voxel_size
                                          : report.grid.dimensions[0];
    const double env_y = voxel_size > 0.0 ? static_cast<double>(coverage_y) * voxel_size
                                          : report.grid.dimensions[1];
    const double env_z = voxel_size > 0.0 ? static_cast<double>(coverage_z) * voxel_size
                                          : report.grid.dimensions[2];

    const std::size_t dictionary_entries = report.dictionary.num_patterns;
    const std::size_t dictionary_bytes = report.dictionary.size;

    const std::size_t index_entries = report.indices.total_blocks;
    const std::size_t index_bytes = report.indices.size;

    std::size_t compressed_bytes = report.statistics.compressed_data_size;
    if (compressed_bytes == 0) {
        compressed_bytes = dictionary_bytes + index_bytes;
    }

    return computeFromCommon(total_voxels,
                             env_x,
                             env_y,
                             env_z,
                             dictionary_entries,
                             dictionary_bytes,
                             index_entries,
                             index_bytes,
                             compressed_bytes);
}

}  // namespace vq_occupancy_compressor::utils
