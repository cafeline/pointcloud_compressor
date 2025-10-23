// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_RUNTIME_RUNTIME_API_HPP
#define POINTCLOUD_COMPRESSOR_RUNTIME_RUNTIME_API_HPP

#include <cstddef>
#include <cstdint>

extern "C" {

struct PCCRuntimeHandle;

struct PCCCompressionRequest {
    const char* input_file;
    double voxel_size;
    int32_t block_size;
    bool use_8bit_indices;
    int32_t min_points_threshold;
    bool save_hdf5;
    const char* hdf5_output_path;
    bool save_raw_hdf5;
    const char* raw_hdf5_output_path;
    double bounding_box_margin_ratio;
};

struct PCCCompressionStatistics {
    double compression_ratio;
    uint32_t original_point_count;
    uint32_t compressed_data_size;
};

struct PCCPatternDictionaryData {
    const uint8_t* data;
    std::size_t size;
    uint32_t num_patterns;
    uint32_t pattern_size_bytes;
};

struct PCCBlockIndicesData {
    const uint8_t* data;
    std::size_t size;
    uint8_t index_bit_size;
    uint32_t total_blocks;
};

struct PCCGridMetadata {
    double dimensions[3];
    double origin[3];
    uint32_t blocks_per_axis[3];
    double voxel_size;
};

struct PCCOccupancyGridView {
    const uint8_t* occupancy;
    std::size_t size;
    uint32_t dimensions[3];
    float origin[3];
};

struct PCCCompressionReport {
    bool success;
    const char* error_message;
    PCCCompressionStatistics statistics;
    PCCPatternDictionaryData dictionary;
    PCCBlockIndicesData indices;
    PCCGridMetadata grid;
    PCCOccupancyGridView occupancy;
    uint64_t max_index;
};

PCCRuntimeHandle* pcc_runtime_create();
void pcc_runtime_destroy(PCCRuntimeHandle* handle);

PCCCompressionReport pcc_runtime_compress(PCCRuntimeHandle* handle,
                                          const PCCCompressionRequest* request);

void pcc_runtime_release_report(PCCRuntimeHandle* handle, PCCCompressionReport* report);

}  // extern "C"

#endif  // POINTCLOUD_COMPRESSOR_RUNTIME_RUNTIME_API_HPP
