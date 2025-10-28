// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_BRIDGE_BRIDGE_HPP
#define VQ_OCCUPANCY_COMPRESSOR_BRIDGE_BRIDGE_HPP

#include <cstddef>
#include <cstdint>

extern "C" {

struct VqoCompressionHandle;

struct PCCCompressionRequest {
    const char* input_file;
    double voxel_size;
    int32_t block_size;
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

VqoCompressionHandle* vqo_handle_create();
void vqo_handle_destroy(VqoCompressionHandle* handle);

PCCCompressionReport vqo_handle_compress(VqoCompressionHandle* handle,
                                         const PCCCompressionRequest* request);

void vqo_handle_release_report(VqoCompressionHandle* handle, PCCCompressionReport* report);

}  

#endif  
