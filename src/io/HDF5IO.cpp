// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <vq_occupancy_compressor/io/HDF5IO.hpp>
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

namespace vq_occupancy_compressor {

bool HDF5IO::write(const std::string& filename, const CompressedMapData& data) {
    auto t0 = std::chrono::high_resolution_clock::now();
    
    std::filesystem::path filepath(filename);
    if (!std::filesystem::exists(filepath.parent_path())) {
        last_error_ = "Directory does not exist: " + filepath.parent_path().string();
        return false;
    }
    
    
    hid_t file_id = H5Fcreate(filename.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    if (file_id < 0) {
        last_error_ = "Failed to create HDF5 file: " + filename;
        return false;
    }
    
    bool success = true;
    
    
    success = success && writeMetadata(file_id, data);
    success = success && writeCompressionParams(file_id, data);
    success = success && writeDictionary(file_id, data);
    success = success && writeCompressedData(file_id, data);
    success = success && writeStatistics(file_id, data);
    
    
    H5Fclose(file_id);
    
    if (!success) {
        
        std::filesystem::remove(filename);
    }
    
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    std::cout << "[PROFILE][HDF5IO] write file='" << filename << "' time=" << ms << " ms" << std::endl;
    return success;
}

bool HDF5IO::read(const std::string& filename, CompressedMapData& data) {
    auto t0 = std::chrono::high_resolution_clock::now();
    
    if (!std::filesystem::exists(filename)) {
        last_error_ = "File does not exist: " + filename;
        return false;
    }
    
    
    hid_t file_id = H5Fopen(filename.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
    if (file_id < 0) {
        last_error_ = "Failed to open HDF5 file: " + filename;
        return false;
    }
    
    bool success = true;
    
    
    success = success && readMetadata(file_id, data);
    success = success && readCompressionParams(file_id, data);
    success = success && readDictionary(file_id, data);
    success = success && readCompressedData(file_id, data);
    success = success && readStatistics(file_id, data);
    
    
    H5Fclose(file_id);
    
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    std::cout << "[PROFILE][HDF5IO] read file='" << filename << "' time=" << ms << " ms" << std::endl;
    return success;
}

bool HDF5IO::isValidHDF5(const std::string& filename) const {
    if (!std::filesystem::exists(filename)) {
        return false;
    }
    
    htri_t is_hdf5 = H5Fis_hdf5(filename.c_str());
    return is_hdf5 > 0;
}

bool HDF5IO::writeRawVoxelGrid(const std::string& filename, const RawVoxelGridData& data) {
    auto t0 = std::chrono::high_resolution_clock::now();
    
    std::filesystem::path filepath(filename);
    if (!std::filesystem::exists(filepath.parent_path())) {
        last_error_ = "Directory does not exist: " + filepath.parent_path().string();
        return false;
    }

    const size_t total_voxels = static_cast<size_t>(data.dim_x) *
                                static_cast<size_t>(data.dim_y) *
                                static_cast<size_t>(data.dim_z);
    if (total_voxels == 0) {
        last_error_ = "Raw voxel grid dimensions must be non-zero";
        return false;
    }

    if (data.voxel_values.size() != total_voxels) {
        std::ostringstream oss;
        oss << "voxel_values size (" << data.voxel_values.size()
            << ") does not match grid dimensions product (" << total_voxels << ")";
        last_error_ = oss.str();
        return false;
    }

    
    hid_t file_id = H5Fcreate(filename.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    if (file_id < 0) {
        last_error_ = "Failed to create HDF5 file: " + filename;
        return false;
    }

    bool ok = true;
    
    hid_t group_id = H5Gcreate2(file_id, "/raw_voxel_grid", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    if (group_id < 0) {
        ok = false;
    } else {
        
        if (ok) {
            hsize_t dims1 = 3;
            hid_t space = H5Screate_simple(1, &dims1, NULL);
            hid_t ds = H5Dcreate2(group_id, "dimensions", H5T_NATIVE_UINT32, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
            if (ds >= 0) {
                uint32_t dims_buf[3] = {data.dim_x, data.dim_y, data.dim_z};
                H5Dwrite(ds, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, dims_buf);
                H5Dclose(ds);
            } else ok = false;
            H5Sclose(space);
        }
        
        if (ok) {
            hsize_t one = 1;
            hid_t space = H5Screate_simple(1, &one, NULL);
            hid_t ds = H5Dcreate2(group_id, "voxel_size", H5T_NATIVE_FLOAT, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
            if (ds >= 0) {
                float v = data.voxel_size;
                H5Dwrite(ds, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, &v);
                H5Dclose(ds);
            } else ok = false;
            H5Sclose(space);
        }
        
        if (ok) {
            hsize_t dims1 = 3;
            hid_t space = H5Screate_simple(1, &dims1, NULL);
            hid_t ds = H5Dcreate2(group_id, "origin", H5T_NATIVE_FLOAT, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
            if (ds >= 0) {
                float origin_buf[3] = {data.origin[0], data.origin[1], data.origin[2]};
                H5Dwrite(ds, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, origin_buf);
                H5Dclose(ds);
            } else ok = false;
            H5Sclose(space);
        }
        
        if (ok) {
            hsize_t dims2[2] = {data.occupied_voxels.size(), 3};
            hid_t space = H5Screate_simple(2, dims2, NULL);
            hid_t ds = H5Dcreate2(group_id, "occupied_voxels", H5T_NATIVE_INT32, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
            if (ds >= 0) {
                if (!data.occupied_voxels.empty()) {
                    H5Dwrite(ds, H5T_NATIVE_INT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, data.occupied_voxels.data());
                }
                H5Dclose(ds);
            } else ok = false;
            H5Sclose(space);
        }
        
        if (ok) {
            hsize_t dims3[3] = {
                static_cast<hsize_t>(data.dim_z),
                static_cast<hsize_t>(data.dim_y),
                static_cast<hsize_t>(data.dim_x)
            };
            hid_t space = H5Screate_simple(3, dims3, NULL);
            if (space >= 0) {
                hid_t ds = H5Dcreate2(group_id, "voxel_values", H5T_NATIVE_UINT8, space,
                                      H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
                if (ds >= 0) {
                    if (!data.voxel_values.empty()) {
                        H5Dwrite(ds, H5T_NATIVE_UINT8, H5S_ALL, H5S_ALL, H5P_DEFAULT, data.voxel_values.data());
                    }
                    H5Dclose(ds);
                } else {
                    ok = false;
                    last_error_ = "Failed to create dataset: /raw_voxel_grid/voxel_values";
                }
                H5Sclose(space);
            } else {
                ok = false;
                last_error_ = "Failed to allocate dataspace for voxel_values";
            }
        }
        H5Gclose(group_id);
    }

    H5Fclose(file_id);
    if (!ok) {
        std::filesystem::remove(filename);
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    std::cout << "[PROFILE][HDF5IO] write raw grid file='" << filename << "' time=" << ms << " ms" << std::endl;
    return ok;
}

bool HDF5IO::createGroup(hid_t file_id, const std::string& group_name) {
    hid_t group_id = H5Gcreate2(file_id, group_name.c_str(), 
                                H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    if (group_id < 0) {
        last_error_ = "Failed to create group: " + group_name;
        return false;
    }
    H5Gclose(group_id);
    return true;
}

bool HDF5IO::writeStringAttribute(hid_t loc_id, const std::string& name, const std::string& value) {
    hid_t datatype = H5Tcopy(H5T_C_S1);
    H5Tset_size(datatype, value.size() + 1);
    H5Tset_strpad(datatype, H5T_STR_NULLTERM);
    
    hid_t dataspace = H5Screate(H5S_SCALAR);
    hid_t attribute = H5Acreate2(loc_id, name.c_str(), datatype, dataspace,
                                 H5P_DEFAULT, H5P_DEFAULT);
    
    if (attribute < 0) {
        H5Sclose(dataspace);
        H5Tclose(datatype);
        return false;
    }
    
    herr_t status = H5Awrite(attribute, datatype, value.c_str());
    
    H5Aclose(attribute);
    H5Sclose(dataspace);
    H5Tclose(datatype);
    
    return status >= 0;
}

bool HDF5IO::readStringAttribute(hid_t loc_id, const std::string& name, std::string& value) {
    if (!H5Aexists(loc_id, name.c_str())) {
        return false;
    }
    
    hid_t attribute = H5Aopen(loc_id, name.c_str(), H5P_DEFAULT);
    if (attribute < 0) {
        return false;
    }
    
    hid_t datatype = H5Aget_type(attribute);
    size_t size = H5Tget_size(datatype);
    
    value.resize(size);
    herr_t status = H5Aread(attribute, datatype, &value[0]);
    
    
    size_t null_pos = value.find('\0');
    if (null_pos != std::string::npos) {
        value.resize(null_pos);
    }
    
    H5Tclose(datatype);
    H5Aclose(attribute);
    
    return status >= 0;
}

bool HDF5IO::writeMetadata(hid_t file_id, const CompressedMapData& data) {
    if (!createGroup(file_id, "/metadata")) {
        return false;
    }
    
    hid_t group_id = H5Gopen2(file_id, "/metadata", H5P_DEFAULT);
    if (group_id < 0) {
        return false;
    }
    
    bool success = true;
    success = success && writeStringAttribute(group_id, "version", data.version);
    success = success && writeStringAttribute(group_id, "creation_time", data.creation_time);
    success = success && writeStringAttribute(group_id, "frame_id", data.frame_id);
    success = success && writeStringAttribute(group_id, "compression_method", data.compression_method);
    
    H5Gclose(group_id);
    return success;
}

bool HDF5IO::readMetadata(hid_t file_id, CompressedMapData& data) {
    if (!H5Lexists(file_id, "/metadata", H5P_DEFAULT)) {
        last_error_ = "Metadata group not found";
        return false;
    }
    
    hid_t group_id = H5Gopen2(file_id, "/metadata", H5P_DEFAULT);
    if (group_id < 0) {
        return false;
    }
    
    readStringAttribute(group_id, "version", data.version);
    readStringAttribute(group_id, "creation_time", data.creation_time);
    readStringAttribute(group_id, "frame_id", data.frame_id);
    readStringAttribute(group_id, "compression_method", data.compression_method);
    
    H5Gclose(group_id);
    return true;
}

bool HDF5IO::writeCompressionParams(hid_t file_id, const CompressedMapData& data) {
    if (!createGroup(file_id, "/compression_params")) {
        return false;
    }
    
    hid_t group_id = H5Gopen2(file_id, "/compression_params", H5P_DEFAULT);
    if (group_id < 0) {
        return false;
    }
    
    
    hsize_t dims = 1;
    hid_t dataspace = H5Screate_simple(1, &dims, NULL);
    
    
    hid_t dataset = H5Dcreate2(group_id, "voxel_size", H5T_NATIVE_FLOAT, dataspace,
                               H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(dataset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.voxel_size);
    H5Dclose(dataset);
    
    
    dataset = H5Dcreate2(group_id, "dictionary_size", H5T_NATIVE_UINT32, dataspace,
                         H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.dictionary_size);
    H5Dclose(dataset);
    
    
    dataset = H5Dcreate2(group_id, "pattern_bits", H5T_NATIVE_UINT32, dataspace,
                         H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.pattern_bits);
    H5Dclose(dataset);
    
    
    dataset = H5Dcreate2(group_id, "block_size", H5T_NATIVE_UINT32, dataspace,
                         H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.block_size);
    H5Dclose(dataset);

    
    uint32_t bit_width = static_cast<uint32_t>(data.block_index_bit_width);
    dataset = H5Dcreate2(group_id, "block_index_bit_width", H5T_NATIVE_UINT32, dataspace,
                         H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &bit_width);
    H5Dclose(dataset);
    
    
    H5Sclose(dataspace);
    hsize_t vec_dims = 3;
    dataspace = H5Screate_simple(1, &vec_dims, NULL);
    dataset = H5Dcreate2(group_id, "grid_origin", H5T_NATIVE_FLOAT, dataspace,
                         H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    float origin_buf[3] = {data.grid_origin[0], data.grid_origin[1], data.grid_origin[2]};
    H5Dwrite(dataset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, origin_buf);
    H5Dclose(dataset);

    
    H5Sclose(dataspace);
    dataspace = H5Screate_simple(1, &vec_dims, NULL);
    dataset = H5Dcreate2(group_id, "grid_dimensions", H5T_NATIVE_INT32, dataspace,
                         H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    int32_t dim_buf[3] = {data.grid_dimensions[0], data.grid_dimensions[1], data.grid_dimensions[2]};
    H5Dwrite(dataset, H5T_NATIVE_INT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, dim_buf);
    H5Dclose(dataset);
    
    H5Sclose(dataspace);
    H5Gclose(group_id);
    return true;
}

bool HDF5IO::readCompressionParams(hid_t file_id, CompressedMapData& data) {
    if (!H5Lexists(file_id, "/compression_params", H5P_DEFAULT)) {
        last_error_ = "Compression params group not found";
        return false;
    }
    
    hid_t group_id = H5Gopen2(file_id, "/compression_params", H5P_DEFAULT);
    if (group_id < 0) {
        return false;
    }
    
    
    hid_t dataset;
    
    
    if (H5Lexists(group_id, "voxel_size", H5P_DEFAULT)) {
        dataset = H5Dopen2(group_id, "voxel_size", H5P_DEFAULT);
        H5Dread(dataset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.voxel_size);
        H5Dclose(dataset);
    }
    
    
    if (H5Lexists(group_id, "dictionary_size", H5P_DEFAULT)) {
        dataset = H5Dopen2(group_id, "dictionary_size", H5P_DEFAULT);
        H5Dread(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.dictionary_size);
        H5Dclose(dataset);
    }
    
    
    if (H5Lexists(group_id, "pattern_bits", H5P_DEFAULT)) {
        dataset = H5Dopen2(group_id, "pattern_bits", H5P_DEFAULT);
        H5Dread(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.pattern_bits);
        H5Dclose(dataset);
    }
    
    
    if (H5Lexists(group_id, "block_size", H5P_DEFAULT)) {
        dataset = H5Dopen2(group_id, "block_size", H5P_DEFAULT);
        H5Dread(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.block_size);
        H5Dclose(dataset);
    }

    
    if (H5Lexists(group_id, "block_index_bit_width", H5P_DEFAULT)) {
        uint32_t bit_width = 0;
        dataset = H5Dopen2(group_id, "block_index_bit_width", H5P_DEFAULT);
        H5Dread(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &bit_width);
        H5Dclose(dataset);
        if (bit_width >= 1 && bit_width <= 32) {
            data.block_index_bit_width = static_cast<uint8_t>(bit_width);
        }
    }
    
    
    if (H5Lexists(group_id, "grid_origin", H5P_DEFAULT)) {
        dataset = H5Dopen2(group_id, "grid_origin", H5P_DEFAULT);
        float origin_buf[3] = {0.0f, 0.0f, 0.0f};
        H5Dread(dataset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, origin_buf);
        data.grid_origin[0] = origin_buf[0];
        data.grid_origin[1] = origin_buf[1];
        data.grid_origin[2] = origin_buf[2];
        H5Dclose(dataset);
    }

    
    if (H5Lexists(group_id, "grid_dimensions", H5P_DEFAULT)) {
        dataset = H5Dopen2(group_id, "grid_dimensions", H5P_DEFAULT);
        int32_t dims_buf[3] = {0, 0, 0};
        H5Dread(dataset, H5T_NATIVE_INT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, dims_buf);
        data.grid_dimensions[0] = dims_buf[0];
        data.grid_dimensions[1] = dims_buf[1];
        data.grid_dimensions[2] = dims_buf[2];
        H5Dclose(dataset);
    }
    
    H5Gclose(group_id);
    return true;
}

bool HDF5IO::writeDictionary(hid_t file_id, const CompressedMapData& data) {
    if (!createGroup(file_id, "/dictionary")) {
        return false;
    }
    
    hid_t group_id = H5Gopen2(file_id, "/dictionary", H5P_DEFAULT);
    if (group_id < 0) {
        return false;
    }
    
    
    hsize_t dims = 1;
    hid_t dataspace = H5Screate_simple(1, &dims, NULL);
    hid_t dataset = H5Dcreate2(group_id, "pattern_length", H5T_NATIVE_UINT32, dataspace,
                               H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.pattern_length);
    H5Dclose(dataset);
    H5Sclose(dataspace);
    
    
    if (!data.dictionary_patterns.empty()) {
        hsize_t array_dims = data.dictionary_patterns.size();
        dataspace = H5Screate_simple(1, &array_dims, NULL);
        dataset = H5Dcreate2(group_id, "patterns", H5T_NATIVE_UINT8, dataspace,
                             H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
        H5Dwrite(dataset, H5T_NATIVE_UINT8, H5S_ALL, H5S_ALL, H5P_DEFAULT, 
                data.dictionary_patterns.data());
        H5Dclose(dataset);
        H5Sclose(dataspace);
    }
    
    H5Gclose(group_id);
    return true;
}

bool HDF5IO::readDictionary(hid_t file_id, CompressedMapData& data) {
    if (!H5Lexists(file_id, "/dictionary", H5P_DEFAULT)) {
        
        return true;
    }
    
    hid_t group_id = H5Gopen2(file_id, "/dictionary", H5P_DEFAULT);
    if (group_id < 0) {
        return false;
    }
    
    
    if (H5Lexists(group_id, "pattern_length", H5P_DEFAULT)) {
        hid_t dataset = H5Dopen2(group_id, "pattern_length", H5P_DEFAULT);
        H5Dread(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.pattern_length);
        H5Dclose(dataset);
    }
    
    
    if (H5Lexists(group_id, "patterns", H5P_DEFAULT)) {
        hid_t dataset = H5Dopen2(group_id, "patterns", H5P_DEFAULT);
        hid_t dataspace = H5Dget_space(dataset);
        
        hsize_t dims[1];
        H5Sget_simple_extent_dims(dataspace, dims, NULL);
        
        data.dictionary_patterns.resize(dims[0]);
        H5Dread(dataset, H5T_NATIVE_UINT8, H5S_ALL, H5S_ALL, H5P_DEFAULT,
               data.dictionary_patterns.data());
        
        H5Sclose(dataspace);
        H5Dclose(dataset);
    }
    
    H5Gclose(group_id);
    return true;
}

bool HDF5IO::writeCompressedData(hid_t file_id, const CompressedMapData& data) {
    if (!createGroup(file_id, "/compressed_data")) {
        last_error_ = "Failed to create /compressed_data group";
        return false;
    }

    hid_t group_id = H5Gopen2(file_id, "/compressed_data", H5P_DEFAULT);
    if (group_id < 0) {
        last_error_ = "Failed to open /compressed_data group";
        return false;
    }

    auto close_group = [&]() {
        if (group_id >= 0) {
            H5Gclose(group_id);
            group_id = -1;
        }
    };

    const bool has_block_grid = !data.block_indices.empty() &&
                                data.block_dims[0] > 0 &&
                                data.block_dims[1] > 0 &&
                                data.block_dims[2] > 0;

    if (has_block_grid) {
        const size_t expected_size = static_cast<size_t>(data.block_dims[0]) *
                                     static_cast<size_t>(data.block_dims[1]) *
                                     static_cast<size_t>(data.block_dims[2]);

        if (data.block_indices.size() != expected_size) {
            close_group();
            last_error_ = "block_indices size does not match block_dims product";
            return false;
        }

        uint8_t configured_bit_width = data.block_index_bit_width > 0 ? data.block_index_bit_width : 1;
        uint32_t max_value = 0;
        for (size_t i = 0; i < expected_size; ++i) {
            max_value = std::max(max_value, data.block_indices[i]);
        }
        auto computeBitWidth = [](uint32_t value) -> uint8_t {
            uint8_t bits = 0;
            while (value > 0) {
                ++bits;
                value >>= 1;
            }
            return bits == 0 ? 1 : bits;
        };
        const uint8_t required_bit_width = computeBitWidth(max_value);
        if (configured_bit_width < required_bit_width) {
            close_group();
            last_error_ = "block_index_bit_width is too small to encode block indices";
            return false;
        }
        if (configured_bit_width > 32) {
            close_group();
            last_error_ = "block_index_bit_width greater than 32 is not supported";
            return false;
        }

        const uint8_t bit_width = configured_bit_width;

        const uint64_t max_allowed = (bit_width == 32)
            ? 0xFFFFFFFFULL
            : ((1ULL << bit_width) - 1ULL);
        for (size_t i = 0; i < expected_size; ++i) {
            if (static_cast<uint64_t>(data.block_indices[i]) > max_allowed) {
                close_group();
                last_error_ = "block index value exceeds representable range for configured bit width";
                return false;
            }
        }

        const size_t packed_size = (expected_size * bit_width + 7) / 8;
        hsize_t dims = static_cast<hsize_t>(packed_size);
        hid_t dataspace = H5Screate_simple(1, &dims, nullptr);
        if (dataspace < 0) {
            close_group();
            last_error_ = "Failed to create dataspace for block_indices";
            return false;
        }

        std::vector<uint8_t> packed(packed_size, 0);
        for (size_t i = 0; i < expected_size; ++i) {
            const uint32_t value = data.block_indices[i];
            for (uint8_t bit = 0; bit < bit_width; ++bit) {
                const size_t absolute_bit = i * static_cast<size_t>(bit_width) + bit;
                const size_t byte_index = absolute_bit >> 3U;
                const size_t bit_index = absolute_bit & 7U;
                if (((value >> bit) & 0x1U) != 0U) {
                    packed[byte_index] = static_cast<uint8_t>(packed[byte_index] | (1U << bit_index));
                }
            }
        }

        hid_t dataset = H5Dcreate2(group_id, "block_indices", H5T_NATIVE_UINT8, dataspace,
                                   H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
        if (dataset < 0) {
            H5Sclose(dataspace);
            close_group();
            last_error_ = "Failed to create block_indices dataset";
            return false;
        }

        herr_t status = H5Dwrite(dataset, H5T_NATIVE_UINT8, H5S_ALL, H5S_ALL, H5P_DEFAULT, packed.data());
        H5Dclose(dataset);
        H5Sclose(dataspace);
        if (status < 0) {
            close_group();
            last_error_ = "Failed to write block_indices dataset";
            return false;
        }
    }

    
    {
        hsize_t dims = 3;
        hid_t dataspace = H5Screate_simple(1, &dims, nullptr);
        if (dataspace >= 0) {
            int32_t buffer[3] = {data.block_dims[0], data.block_dims[1], data.block_dims[2]};
            hid_t dataset = H5Dcreate2(group_id, "block_dims", H5T_NATIVE_INT32, dataspace,
                                       H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
            if (dataset >= 0) {
                H5Dwrite(dataset, H5T_NATIVE_INT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, buffer);
                H5Dclose(dataset);
            }
            H5Sclose(dataspace);
        }
    }

    
    {
        hsize_t dims = 1;
        hid_t dataspace = H5Screate_simple(1, &dims, nullptr);
        if (dataspace >= 0) {
            hid_t dataset = H5Dcreate2(group_id, "point_count", H5T_NATIVE_UINT64, dataspace,
                                       H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
            if (dataset >= 0) {
                uint64_t count = data.compressed_voxels;
                H5Dwrite(dataset, H5T_NATIVE_UINT64, H5S_ALL, H5S_ALL, H5P_DEFAULT, &count);
                H5Dclose(dataset);
            }
            H5Sclose(dataspace);
        }
    }

    close_group();
    return true;
}

bool HDF5IO::readCompressedData(hid_t file_id, CompressedMapData& data) {
    if (!H5Lexists(file_id, "/compressed_data", H5P_DEFAULT)) {
        
        return true;
    }

    hid_t group_id = H5Gopen2(file_id, "/compressed_data", H5P_DEFAULT);
    if (group_id < 0) {
        return false;
    }

    if (!H5Lexists(group_id, "block_indices", H5P_DEFAULT)) {
        last_error_ = "Missing block_indices dataset in /compressed_data";
        H5Gclose(group_id);
        return false;
    }

    std::vector<uint8_t> packed_block_indices;
    size_t packed_size = 0;

    {
        hid_t dataset = H5Dopen2(group_id, "block_indices", H5P_DEFAULT);
        if (dataset < 0) {
            last_error_ = "Failed to open block_indices dataset";
            H5Gclose(group_id);
            return false;
        }

        hid_t dataspace = H5Dget_space(dataset);
        hsize_t dims[1];
        H5Sget_simple_extent_dims(dataspace, dims, NULL);
        packed_size = static_cast<size_t>(dims[0]);

        hid_t dtype = H5Dget_type(dataset);
        const size_t type_size = H5Tget_size(dtype);
        bool ok = (type_size == sizeof(uint8_t));
        if (ok) {
            packed_block_indices.resize(packed_size);
            ok = H5Dread(dataset, H5T_NATIVE_UINT8, H5S_ALL, H5S_ALL, H5P_DEFAULT,
                         packed_block_indices.data()) >= 0;
        }

        H5Tclose(dtype);
        H5Sclose(dataspace);
        H5Dclose(dataset);

        if (!ok) {
            last_error_ = "Failed to read block_indices dataset";
            H5Gclose(group_id);
            return false;
        }
    }

    if (!H5Lexists(group_id, "block_dims", H5P_DEFAULT)) {
        last_error_ = "Missing block_dims dataset";
        H5Gclose(group_id);
        return false;
    }

    size_t total_blocks = 0;
    {
        hid_t dataset = H5Dopen2(group_id, "block_dims", H5P_DEFAULT);
        int32_t buffer[3] = {0, 0, 0};
        H5Dread(dataset, H5T_NATIVE_INT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, buffer);
        H5Dclose(dataset);
        for (int i = 0; i < 3; ++i) {
            data.block_dims[i] = buffer[i];
        }
        total_blocks = static_cast<size_t>(data.block_dims[0]) *
                       static_cast<size_t>(data.block_dims[1]) *
                       static_cast<size_t>(data.block_dims[2]);
    }

    const size_t bits_per_index = data.block_index_bit_width > 0 ? data.block_index_bit_width : 1;
    if (bits_per_index > 32) {
        H5Gclose(group_id);
        last_error_ = "Unsupported block_index_bit_width > 32 in file";
        return false;
    }

    if (total_blocks == 0) {
        data.block_indices.clear();
    } else {
        const size_t expected_packed = (total_blocks * bits_per_index + 7) / 8;
        if (packed_block_indices.size() != expected_packed) {
            H5Gclose(group_id);
            last_error_ = "Packed block_indices size mismatch";
            return false;
        }

        data.block_indices.assign(total_blocks, 0);
        for (size_t i = 0; i < total_blocks; ++i) {
            uint32_t value = 0;
            for (size_t bit = 0; bit < bits_per_index; ++bit) {
                const size_t absolute_bit = i * bits_per_index + bit;
                const size_t byte_index = absolute_bit >> 3U;
                const size_t bit_index = absolute_bit & 7U;
                const uint8_t byte = packed_block_indices[byte_index];
                const uint8_t bit_value = static_cast<uint8_t>((byte >> bit_index) & 0x1U);
                value |= static_cast<uint32_t>(bit_value) << bit;
            }
            data.block_indices[i] = value;
        }
    }

    H5Gclose(group_id);
    return true;
}

bool HDF5IO::writeStatistics(hid_t file_id, const CompressedMapData& data) {
    if (!createGroup(file_id, "/statistics")) {
        return false;
    }
    
    hid_t group_id = H5Gopen2(file_id, "/statistics", H5P_DEFAULT);
    if (group_id < 0) {
        return false;
    }
    
    hsize_t dims = 1;
    hid_t dataspace = H5Screate_simple(1, &dims, NULL);
    
    
    hid_t dataset = H5Dcreate2(group_id, "original_points", H5T_NATIVE_UINT64, dataspace,
                               H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(dataset, H5T_NATIVE_UINT64, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.original_points);
    H5Dclose(dataset);
    
    dataset = H5Dcreate2(group_id, "compressed_voxels", H5T_NATIVE_UINT64, dataspace,
                         H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(dataset, H5T_NATIVE_UINT64, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.compressed_voxels);
    H5Dclose(dataset);
    
    dataset = H5Dcreate2(group_id, "compression_ratio", H5T_NATIVE_DOUBLE, dataspace,
                         H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(dataset, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.compression_ratio);
    H5Dclose(dataset);
    
    H5Sclose(dataspace);
    
    
    hsize_t bbox_dims[2] = {2, 3};
    dataspace = H5Screate_simple(2, bbox_dims, NULL);
    dataset = H5Dcreate2(group_id, "bounding_box", H5T_NATIVE_DOUBLE, dataspace,
                         H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    
    double bbox[6];
    for (int i = 0; i < 3; i++) {
        bbox[i] = data.bounding_box_min[i];
        bbox[3 + i] = data.bounding_box_max[i];
    }
    H5Dwrite(dataset, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, bbox);
    H5Dclose(dataset);
    H5Sclose(dataspace);

    H5Gclose(group_id);
    return true;
}

bool HDF5IO::readStatistics(hid_t file_id, CompressedMapData& data) {
    if (!H5Lexists(file_id, "/statistics", H5P_DEFAULT)) {
        
        return true;
    }
    
    hid_t group_id = H5Gopen2(file_id, "/statistics", H5P_DEFAULT);
    if (group_id < 0) {
        return false;
    }
    
    
    hid_t dataset;
    
    if (H5Lexists(group_id, "original_points", H5P_DEFAULT)) {
        dataset = H5Dopen2(group_id, "original_points", H5P_DEFAULT);
        H5Dread(dataset, H5T_NATIVE_UINT64, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.original_points);
        H5Dclose(dataset);
    }
    
    if (H5Lexists(group_id, "compressed_voxels", H5P_DEFAULT)) {
        dataset = H5Dopen2(group_id, "compressed_voxels", H5P_DEFAULT);
        H5Dread(dataset, H5T_NATIVE_UINT64, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.compressed_voxels);
        H5Dclose(dataset);
    }
    
    if (H5Lexists(group_id, "compression_ratio", H5P_DEFAULT)) {
        dataset = H5Dopen2(group_id, "compression_ratio", H5P_DEFAULT);
        H5Dread(dataset, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.compression_ratio);
        H5Dclose(dataset);
    }
    
    
    if (H5Lexists(group_id, "bounding_box", H5P_DEFAULT)) {
        dataset = H5Dopen2(group_id, "bounding_box", H5P_DEFAULT);
        double bbox[6];
        H5Dread(dataset, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, bbox);
        for (int i = 0; i < 3; i++) {
            data.bounding_box_min[i] = bbox[i];
            data.bounding_box_max[i] = bbox[3 + i];
        }
        H5Dclose(dataset);
    }

    H5Gclose(group_id);
    return true;
}

} 
