// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <array>
#include <limits>
#include <hdf5.h>
#include <hdf5_hl.h>

namespace vq_occupancy_compressor {

struct CompressedMapData {
    std::string version = "1.0.0";
    std::string creation_time;
    std::string frame_id = "map";
    std::string compression_method = "voxel_dictionary";
    
    float voxel_size = 0.1f;
    uint32_t dictionary_size = 256;
    uint32_t pattern_bits = 8;
    uint32_t block_size = 8;
    std::array<float, 3> grid_origin = {0.0f, 0.0f, 0.0f};
    std::array<int32_t, 3> grid_dimensions = {0, 0, 0};
    
    std::vector<uint8_t> dictionary_patterns;
    uint32_t pattern_length = 512;
    
    std::array<int32_t, 3> block_dims = {0, 0, 0};
    uint8_t block_index_bit_width = 1;
    uint64_t block_index_sentinel = 0;
    std::vector<uint32_t> block_indices;

    uint64_t original_points = 0;
    uint64_t compressed_voxels = 0;
    double compression_ratio = 0.0;
    std::array<double, 3> bounding_box_min = {0, 0, 0};
    std::array<double, 3> bounding_box_max = {0, 0, 0};
};

class HDF5IO {
public:
    HDF5IO() = default;
    ~HDF5IO() = default;
    
    bool write(const std::string& filename, const CompressedMapData& data);
    
    bool read(const std::string& filename, CompressedMapData& data);
    
    bool isValidHDF5(const std::string& filename) const;
    
    std::string getLastError() const { return last_error_; }

    struct RawVoxelGridData {
        uint32_t dim_x = 0, dim_y = 0, dim_z = 0;
        float voxel_size = 0.0f;
        std::array<float,3> origin{0.0f, 0.0f, 0.0f};
        std::vector<uint8_t> voxel_values;
        std::vector<std::array<int32_t,3>> occupied_voxels;
    };

    bool writeRawVoxelGrid(const std::string& filename, const RawVoxelGridData& data);

private:
    std::string last_error_;
    
    bool writeMetadata(hid_t file_id, const CompressedMapData& data);
    bool writeCompressionParams(hid_t file_id, const CompressedMapData& data);
    bool writeDictionary(hid_t file_id, const CompressedMapData& data);
    bool writeCompressedData(hid_t file_id, const CompressedMapData& data);
    bool writeStatistics(hid_t file_id, const CompressedMapData& data);
    
    bool readMetadata(hid_t file_id, CompressedMapData& data);
    bool readCompressionParams(hid_t file_id, CompressedMapData& data);
    bool readDictionary(hid_t file_id, CompressedMapData& data);
    bool readCompressedData(hid_t file_id, CompressedMapData& data);
    bool readStatistics(hid_t file_id, CompressedMapData& data);
    
    bool createGroup(hid_t file_id, const std::string& group_name);
    bool writeStringAttribute(hid_t loc_id, const std::string& name, const std::string& value);
    bool readStringAttribute(hid_t loc_id, const std::string& name, std::string& value);
};

}
