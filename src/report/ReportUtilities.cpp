// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "vq_occupancy_compressor/report/ReportUtilities.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

#include "vq_occupancy_compressor/common/CompressionArtifacts.hpp"
#include "vq_occupancy_compressor/common/CompressionDataUtils.hpp"

namespace {

PCCCompressionReport makeReportSkeleton() {
    PCCCompressionReport report{};
    report.success = false;
    report.error_message = nullptr;
    report.statistics = {};
    report.dictionary = {};
    report.indices = {};
    report.grid = {};
    report.occupancy = {};
    report.max_index = 0;
    return report;
}

}  

namespace vq_occupancy_compressor {

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

}  

namespace vq_occupancy_compressor::io {

namespace {

std::vector<uint32_t> decodeIndices(const PCCCompressionReport& report) {
    std::vector<uint32_t> indices;
    if (!report.indices.data || report.indices.size == 0) {
        return indices;
    }

    const uint8_t* data = report.indices.data;
    const std::size_t total_blocks = report.indices.total_blocks;
    switch (report.indices.index_bit_size) {
        case 8: {
            indices.reserve(total_blocks);
            for (std::size_t i = 0; i < report.indices.size; ++i) {
                indices.push_back(static_cast<uint32_t>(data[i]));
            }
            break;
        }
        case 16: {
            indices.reserve(total_blocks);
            for (std::size_t i = 0; i + 1 < report.indices.size; i += 2) {
                uint32_t value = static_cast<uint32_t>(data[i]) |
                                 (static_cast<uint32_t>(data[i + 1]) << 8);
                indices.push_back(value);
            }
            break;
        }
        case 32: {
            indices.reserve(total_blocks);
            for (std::size_t i = 0; i + 3 < report.indices.size; i += 4) {
                uint32_t value = static_cast<uint32_t>(data[i]) |
                                 (static_cast<uint32_t>(data[i + 1]) << 8) |
                                 (static_cast<uint32_t>(data[i + 2]) << 16) |
                                 (static_cast<uint32_t>(data[i + 3]) << 24);
                indices.push_back(value);
            }
            break;
        }
        case 64: {
            indices.reserve(total_blocks);
            for (std::size_t i = 0; i + 7 < report.indices.size; i += 8) {
                uint64_t value = static_cast<uint64_t>(data[i]) |
                                 (static_cast<uint64_t>(data[i + 1]) << 8) |
                                 (static_cast<uint64_t>(data[i + 2]) << 16) |
                                 (static_cast<uint64_t>(data[i + 3]) << 24) |
                                 (static_cast<uint64_t>(data[i + 4]) << 32) |
                                 (static_cast<uint64_t>(data[i + 5]) << 40) |
                                 (static_cast<uint64_t>(data[i + 6]) << 48) |
                                 (static_cast<uint64_t>(data[i + 7]) << 56);
                indices.push_back(static_cast<uint32_t>(value));
            }
            break;
        }
        default:
            break;
    }
    return indices;
}

}  

PCCCompressionReport CompressionReportBuilder::build(const CompressionResult& result,
                                                     const PCCCompressionRequest& request,
                                                     std::vector<uint8_t>& dictionary_buffer,
                                                     std::vector<uint8_t>& indices_buffer,
                                                     std::vector<uint8_t>& occupancy_buffer,
                                                     std::string& transient_error_message) const {
    dictionary_buffer = vq_occupancy_compressor::common::flattenDictionaryPatterns(result.pattern_dictionary);
    indices_buffer = vq_occupancy_compressor::common::packBlockIndices(result.block_indices, static_cast<uint8_t>(result.index_bit_size));
    occupancy_buffer = vq_occupancy_compressor::common::buildOccupancyMask(result.voxel_grid);
    transient_error_message.clear();

    PCCCompressionReport report = makeReportSkeleton();
    report.success = true;

    report.dictionary.num_patterns = static_cast<uint32_t>(result.pattern_dictionary.size());
    const uint32_t pattern_bits = static_cast<uint32_t>(request.block_size) *
                                  static_cast<uint32_t>(request.block_size) *
                                  static_cast<uint32_t>(request.block_size);
    report.dictionary.pattern_size_bytes = (pattern_bits + 7) / 8;
    report.dictionary.size = dictionary_buffer.size();
    report.dictionary.data = dictionary_buffer.empty() ? nullptr : dictionary_buffer.data();

    report.indices.data = indices_buffer.empty() ? nullptr : indices_buffer.data();
    report.indices.size = indices_buffer.size();
    report.indices.index_bit_size = static_cast<uint8_t>(result.index_bit_size);
    report.indices.total_blocks = static_cast<uint32_t>(result.num_blocks);

    report.grid.dimensions[0] = result.grid_dimensions.x;
    report.grid.dimensions[1] = result.grid_dimensions.y;
    report.grid.dimensions[2] = result.grid_dimensions.z;

    report.grid.origin[0] = result.grid_origin.x;
    report.grid.origin[1] = result.grid_origin.y;
    report.grid.origin[2] = result.grid_origin.z;

    report.grid.blocks_per_axis[0] = result.blocks_count.x;
    report.grid.blocks_per_axis[1] = result.blocks_count.y;
    report.grid.blocks_per_axis[2] = result.blocks_count.z;

    report.grid.voxel_size = result.voxel_size;
    report.occupancy.occupancy = occupancy_buffer.empty() ? nullptr : occupancy_buffer.data();
    report.occupancy.size = occupancy_buffer.size();
    report.occupancy.dimensions[0] = result.voxel_stats.blocks_per_axis.x;
    report.occupancy.dimensions[1] = result.voxel_stats.blocks_per_axis.y;
    report.occupancy.dimensions[2] = result.voxel_stats.blocks_per_axis.z;
    report.occupancy.origin[0] = static_cast<float>(result.grid_origin.x);
    report.occupancy.origin[1] = static_cast<float>(result.grid_origin.y);
    report.occupancy.origin[2] = static_cast<float>(result.grid_origin.z);

    report.statistics.compression_ratio = result.compression_ratio;
    report.statistics.original_point_count = static_cast<uint32_t>(result.point_count);
    report.statistics.compressed_data_size = static_cast<uint32_t>(indices_buffer.size() + dictionary_buffer.size());

    report.max_index = result.max_index;

    return report;
}

vq_occupancy_compressor::CompressedMapData CompressionReportBuilder::toCompressedMapData(
    const CompressionResult& result,
    const PCCCompressionRequest& request,
    const std::vector<uint8_t>& dictionary_buffer,
    const std::vector<uint32_t>& block_indices_u32) const {
    vq_occupancy_compressor::CompressedMapData data;
    data.voxel_size = static_cast<float>(request.voxel_size);
    data.dictionary_size = static_cast<uint32_t>(result.num_unique_patterns);
    data.block_size = static_cast<uint32_t>(result.block_size);

    const uint32_t pattern_bits = static_cast<uint32_t>(result.block_size) *
                                  static_cast<uint32_t>(result.block_size) *
                                  static_cast<uint32_t>(result.block_size);
    data.pattern_bits = pattern_bits;
    data.pattern_length = pattern_bits;

    data.dictionary_patterns.assign(dictionary_buffer.begin(), dictionary_buffer.end());

    float origin_x = 0.0f;
    float origin_y = 0.0f;
    float origin_z = 0.0f;
    result.voxel_grid.getOrigin(origin_x, origin_y, origin_z);
    data.grid_origin = {origin_x, origin_y, origin_z};

    const auto dims = result.voxel_grid.getDimensions();
    data.grid_dimensions = {
        static_cast<int32_t>(dims.x),
        static_cast<int32_t>(dims.y),
        static_cast<int32_t>(dims.z)};

    data.block_dims = {
        static_cast<int32_t>(result.blocks_count.x),
        static_cast<int32_t>(result.blocks_count.y),
        static_cast<int32_t>(result.blocks_count.z)};

    data.block_indices.assign(block_indices_u32.begin(), block_indices_u32.end());

    data.block_index_bit_width = vq_occupancy_compressor::common::bitWidthFromMaxIndex(result.max_index);
    data.block_index_sentinel = 0;

    data.original_points = static_cast<uint64_t>(result.point_count);
    data.compressed_voxels = static_cast<uint64_t>(result.num_blocks) *
                             static_cast<uint64_t>(result.block_size) *
                             static_cast<uint64_t>(result.block_size) *
                             static_cast<uint64_t>(result.block_size);
    data.compression_ratio = result.compression_ratio;

    data.bounding_box_min = {
        result.grid_origin.x,
        result.grid_origin.y,
        result.grid_origin.z};

    data.bounding_box_max = {
        result.grid_origin.x + result.grid_dimensions.x,
        result.grid_origin.y + result.grid_dimensions.y,
        result.grid_origin.z + result.grid_dimensions.z};

    return data;
}

vq_occupancy_compressor::CompressedMapData CompressionReportBuilder::toCompressedMapData(
    const PCCCompressionReport& report,
    double voxel_size,
    int block_size) const {
    vq_occupancy_compressor::CompressedMapData data;
    data.voxel_size = static_cast<float>(voxel_size);
    data.dictionary_size = report.dictionary.num_patterns;
    data.block_size = static_cast<uint32_t>(block_size);

    const uint32_t pattern_bits = report.dictionary.pattern_size_bytes * 8;
    data.pattern_bits = pattern_bits;
    data.pattern_length = pattern_bits;
    if (report.dictionary.data && report.dictionary.size > 0) {
        data.dictionary_patterns.assign(report.dictionary.data,
                                        report.dictionary.data + report.dictionary.size);
    }

    data.grid_origin = {static_cast<float>(report.grid.origin[0]),
                        static_cast<float>(report.grid.origin[1]),
                        static_cast<float>(report.grid.origin[2])};
    data.grid_dimensions = {static_cast<int32_t>(report.grid.dimensions[0]),
                             static_cast<int32_t>(report.grid.dimensions[1]),
                             static_cast<int32_t>(report.grid.dimensions[2])};
    data.block_dims = {static_cast<int32_t>(report.grid.blocks_per_axis[0]),
                       static_cast<int32_t>(report.grid.blocks_per_axis[1]),
                       static_cast<int32_t>(report.grid.blocks_per_axis[2])};

    auto indices = decodeIndices(report);
    data.block_indices = std::move(indices);
    data.block_index_bit_width = report.indices.index_bit_size;
    data.block_index_sentinel = 0;

    data.original_points = report.statistics.original_point_count;
    data.compressed_voxels = static_cast<uint64_t>(report.indices.total_blocks) *
                             static_cast<uint64_t>(block_size) *
                             static_cast<uint64_t>(block_size) *
                             static_cast<uint64_t>(block_size);
    data.compression_ratio = report.statistics.compression_ratio;

    data.bounding_box_min = {report.grid.origin[0], report.grid.origin[1], report.grid.origin[2]};
    data.bounding_box_max = {report.grid.origin[0] + report.grid.dimensions[0],
                             report.grid.origin[1] + report.grid.dimensions[1],
                             report.grid.origin[2] + report.grid.dimensions[2]};

    return data;
}

std::vector<uint8_t> CompressionReportBuilder::extractOccupancy(
    const PCCCompressionReport& report) const {
    if (!report.occupancy.occupancy || report.occupancy.size == 0) {
        return {};
    }
    return std::vector<uint8_t>(report.occupancy.occupancy,
                                report.occupancy.occupancy + report.occupancy.size);
}

}  
