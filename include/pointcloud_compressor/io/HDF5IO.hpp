#pragma once

#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <array>
#include <limits>
#include <hdf5.h>
#include <hdf5_hl.h>

namespace pointcloud_compressor {

struct CompressedMapData {
    // Metadata
    std::string version = "1.0.0";
    std::string creation_time;
    std::string frame_id = "map";
    std::string compression_method = "voxel_dictionary";
    
    // Compression parameters
    float voxel_size = 0.1f;
    uint32_t dictionary_size = 256;
    uint32_t pattern_bits = 8;
    uint32_t block_size = 8;
    // Grid origin in world coordinates (meters)
    std::array<float, 3> grid_origin = {0.0f, 0.0f, 0.0f};
    
    // Dictionary data
    std::vector<uint8_t> dictionary_patterns;  // Flattened pattern data
    uint32_t pattern_length = 512;  // 8x8x8 = 512 bits per pattern
    
    // New block index grid representation
    std::array<int32_t, 3> block_dims = {0, 0, 0};
    uint8_t block_index_bit_width = 1;   // Bits per block index entry
    uint64_t block_index_sentinel = 0;   // 0 for unused blocks
    std::vector<uint32_t> block_indices;  // Flattened block index grid (dictionary pattern IDs)

    // Statistics
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
    
    /**
     * @brief Write compressed map data to HDF5 file
     * @param filename Output HDF5 file path
     * @param data Compressed map data to write
     * @return true if successful, false otherwise
     */
    bool write(const std::string& filename, const CompressedMapData& data);
    
    /**
     * @brief Read compressed map data from HDF5 file
     * @param filename Input HDF5 file path
     * @param data Output compressed map data
     * @return true if successful, false otherwise
     */
    bool read(const std::string& filename, CompressedMapData& data);
    
    /**
     * @brief Check if file exists and is valid HDF5
     * @param filename File path to check
     * @return true if valid HDF5 file
     */
    bool isValidHDF5(const std::string& filename) const;
    
    /**
     * @brief Get last error message
     * @return Error message string
     */
    std::string getLastError() const { return last_error_; }

    // --- Raw voxel grid (pre-compression) I/O ---
    struct RawVoxelGridData {
        uint32_t dim_x = 0, dim_y = 0, dim_z = 0;
        float voxel_size = 0.0f;
        std::array<float,3> origin{0.0f, 0.0f, 0.0f};
        std::vector<uint8_t> voxel_values;                  // flattened occupancy values (z-major, 0-255)
        std::vector<std::array<int32_t,3>> occupied_voxels; // voxel indices (x,y,z) for compatibility
    };

    /**
     * @brief Write raw occupancy voxel grid to HDF5 file under /raw_voxel_grid
     */
    bool writeRawVoxelGrid(const std::string& filename, const RawVoxelGridData& data);

private:
    std::string last_error_;
    
    // Helper methods for writing
    bool writeMetadata(hid_t file_id, const CompressedMapData& data);
    bool writeCompressionParams(hid_t file_id, const CompressedMapData& data);
    bool writeDictionary(hid_t file_id, const CompressedMapData& data);
    bool writeCompressedData(hid_t file_id, const CompressedMapData& data);
    bool writeStatistics(hid_t file_id, const CompressedMapData& data);
    
    // Helper methods for reading
    bool readMetadata(hid_t file_id, CompressedMapData& data);
    bool readCompressionParams(hid_t file_id, CompressedMapData& data);
    bool readDictionary(hid_t file_id, CompressedMapData& data);
    bool readCompressedData(hid_t file_id, CompressedMapData& data);
    bool readStatistics(hid_t file_id, CompressedMapData& data);
    
    // Utility methods
    bool createGroup(hid_t file_id, const std::string& group_name);
    bool writeStringAttribute(hid_t loc_id, const std::string& name, const std::string& value);
    bool readStringAttribute(hid_t loc_id, const std::string& name, std::string& value);
};

} // namespace pointcloud_compressor
