#include <pointcloud_compressor/io/HDF5IO.hpp>
#include <filesystem>
#include <iostream>
#include <sstream>

namespace pointcloud_compressor {

bool HDF5IO::write(const std::string& filename, const CompressedMapData& data) {
    // Check if directory exists
    std::filesystem::path filepath(filename);
    if (!std::filesystem::exists(filepath.parent_path())) {
        last_error_ = "Directory does not exist: " + filepath.parent_path().string();
        return false;
    }
    
    // Create HDF5 file
    hid_t file_id = H5Fcreate(filename.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    if (file_id < 0) {
        last_error_ = "Failed to create HDF5 file: " + filename;
        return false;
    }
    
    bool success = true;
    
    // Write all sections
    success = success && writeMetadata(file_id, data);
    success = success && writeCompressionParams(file_id, data);
    success = success && writeDictionary(file_id, data);
    success = success && writeCompressedData(file_id, data);
    success = success && writeStatistics(file_id, data);
    
    // Close file
    H5Fclose(file_id);
    
    if (!success) {
        // Remove incomplete file
        std::filesystem::remove(filename);
    }
    
    return success;
}

bool HDF5IO::read(const std::string& filename, CompressedMapData& data) {
    // Check if file exists
    if (!std::filesystem::exists(filename)) {
        last_error_ = "File does not exist: " + filename;
        return false;
    }
    
    // Open HDF5 file
    hid_t file_id = H5Fopen(filename.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
    if (file_id < 0) {
        last_error_ = "Failed to open HDF5 file: " + filename;
        return false;
    }
    
    bool success = true;
    
    // Read all sections
    success = success && readMetadata(file_id, data);
    success = success && readCompressionParams(file_id, data);
    success = success && readDictionary(file_id, data);
    success = success && readCompressedData(file_id, data);
    success = success && readStatistics(file_id, data);
    
    // Close file
    H5Fclose(file_id);
    
    return success;
}

bool HDF5IO::isValidHDF5(const std::string& filename) const {
    if (!std::filesystem::exists(filename)) {
        return false;
    }
    
    htri_t is_hdf5 = H5Fis_hdf5(filename.c_str());
    return is_hdf5 > 0;
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
    
    // Remove null terminator if present
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
    
    // Write scalar datasets
    hsize_t dims = 1;
    hid_t dataspace = H5Screate_simple(1, &dims, NULL);
    
    // voxel_size
    hid_t dataset = H5Dcreate2(group_id, "voxel_size", H5T_NATIVE_FLOAT, dataspace,
                               H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(dataset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.voxel_size);
    H5Dclose(dataset);
    
    // dictionary_size
    dataset = H5Dcreate2(group_id, "dictionary_size", H5T_NATIVE_UINT32, dataspace,
                         H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.dictionary_size);
    H5Dclose(dataset);
    
    // pattern_bits
    dataset = H5Dcreate2(group_id, "pattern_bits", H5T_NATIVE_UINT32, dataspace,
                         H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.pattern_bits);
    H5Dclose(dataset);
    
    // block_size
    dataset = H5Dcreate2(group_id, "block_size", H5T_NATIVE_UINT32, dataspace,
                         H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.block_size);
    H5Dclose(dataset);
    
    // grid_origin (vector length 3)
    H5Sclose(dataspace);
    hsize_t origin_dims = 3;
    dataspace = H5Screate_simple(1, &origin_dims, NULL);
    dataset = H5Dcreate2(group_id, "grid_origin", H5T_NATIVE_FLOAT, dataspace,
                         H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    float origin_buf[3] = {data.grid_origin[0], data.grid_origin[1], data.grid_origin[2]};
    H5Dwrite(dataset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, origin_buf);
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
    
    // Read scalar datasets
    hid_t dataset;
    
    // voxel_size
    if (H5Lexists(group_id, "voxel_size", H5P_DEFAULT)) {
        dataset = H5Dopen2(group_id, "voxel_size", H5P_DEFAULT);
        H5Dread(dataset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.voxel_size);
        H5Dclose(dataset);
    }
    
    // dictionary_size
    if (H5Lexists(group_id, "dictionary_size", H5P_DEFAULT)) {
        dataset = H5Dopen2(group_id, "dictionary_size", H5P_DEFAULT);
        H5Dread(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.dictionary_size);
        H5Dclose(dataset);
    }
    
    // pattern_bits
    if (H5Lexists(group_id, "pattern_bits", H5P_DEFAULT)) {
        dataset = H5Dopen2(group_id, "pattern_bits", H5P_DEFAULT);
        H5Dread(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.pattern_bits);
        H5Dclose(dataset);
    }
    
    // block_size
    if (H5Lexists(group_id, "block_size", H5P_DEFAULT)) {
        dataset = H5Dopen2(group_id, "block_size", H5P_DEFAULT);
        H5Dread(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.block_size);
        H5Dclose(dataset);
    }
    
    // grid_origin
    if (H5Lexists(group_id, "grid_origin", H5P_DEFAULT)) {
        dataset = H5Dopen2(group_id, "grid_origin", H5P_DEFAULT);
        float origin_buf[3] = {0.0f, 0.0f, 0.0f};
        H5Dread(dataset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, origin_buf);
        data.grid_origin[0] = origin_buf[0];
        data.grid_origin[1] = origin_buf[1];
        data.grid_origin[2] = origin_buf[2];
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
    
    // Write pattern_length
    hsize_t dims = 1;
    hid_t dataspace = H5Screate_simple(1, &dims, NULL);
    hid_t dataset = H5Dcreate2(group_id, "pattern_length", H5T_NATIVE_UINT32, dataspace,
                               H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.pattern_length);
    H5Dclose(dataset);
    H5Sclose(dataspace);
    
    // Write patterns array
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
        // Dictionary is optional
        return true;
    }
    
    hid_t group_id = H5Gopen2(file_id, "/dictionary", H5P_DEFAULT);
    if (group_id < 0) {
        return false;
    }
    
    // Read pattern_length
    if (H5Lexists(group_id, "pattern_length", H5P_DEFAULT)) {
        hid_t dataset = H5Dopen2(group_id, "pattern_length", H5P_DEFAULT);
        H5Dread(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &data.pattern_length);
        H5Dclose(dataset);
    }
    
    // Read patterns array
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
        return false;
    }
    
    hid_t group_id = H5Gopen2(file_id, "/compressed_data", H5P_DEFAULT);
    if (group_id < 0) {
        return false;
    }
    
    // Write voxel indices
    if (!data.voxel_indices.empty()) {
        hsize_t dims = data.voxel_indices.size();
        hid_t dataspace = H5Screate_simple(1, &dims, NULL);
        hid_t dataset = H5Dcreate2(group_id, "indices", H5T_NATIVE_UINT16, dataspace,
                                   H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
        H5Dwrite(dataset, H5T_NATIVE_UINT16, H5S_ALL, H5S_ALL, H5P_DEFAULT,
                data.voxel_indices.data());
        H5Dclose(dataset);
        H5Sclose(dataspace);
    }
    
    // Write voxel positions
    if (!data.voxel_positions.empty()) {
        hsize_t dims[2] = {data.voxel_positions.size(), 3};
        hid_t dataspace = H5Screate_simple(2, dims, NULL);
        hid_t dataset = H5Dcreate2(group_id, "voxel_positions", H5T_NATIVE_INT32, dataspace,
                                   H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
        H5Dwrite(dataset, H5T_NATIVE_INT32, H5S_ALL, H5S_ALL, H5P_DEFAULT,
                data.voxel_positions.data());
        H5Dclose(dataset);
        H5Sclose(dataspace);
    }
    
    // Write point count
    hsize_t scalar_dims = 1;
    hid_t dataspace = H5Screate_simple(1, &scalar_dims, NULL);
    hid_t dataset = H5Dcreate2(group_id, "point_count", H5T_NATIVE_UINT64, dataspace,
                               H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    uint64_t count = data.voxel_positions.size();
    H5Dwrite(dataset, H5T_NATIVE_UINT64, H5S_ALL, H5S_ALL, H5P_DEFAULT, &count);
    H5Dclose(dataset);
    H5Sclose(dataspace);
    
    H5Gclose(group_id);
    return true;
}

bool HDF5IO::readCompressedData(hid_t file_id, CompressedMapData& data) {
    if (!H5Lexists(file_id, "/compressed_data", H5P_DEFAULT)) {
        // Compressed data is optional
        return true;
    }
    
    hid_t group_id = H5Gopen2(file_id, "/compressed_data", H5P_DEFAULT);
    if (group_id < 0) {
        return false;
    }
    
    // Read voxel indices
    if (H5Lexists(group_id, "indices", H5P_DEFAULT)) {
        hid_t dataset = H5Dopen2(group_id, "indices", H5P_DEFAULT);
        hid_t dataspace = H5Dget_space(dataset);
        
        hsize_t dims[1];
        H5Sget_simple_extent_dims(dataspace, dims, NULL);
        
        data.voxel_indices.resize(dims[0]);
        H5Dread(dataset, H5T_NATIVE_UINT16, H5S_ALL, H5S_ALL, H5P_DEFAULT,
               data.voxel_indices.data());
        
        H5Sclose(dataspace);
        H5Dclose(dataset);
    }
    
    // Read voxel positions
    if (H5Lexists(group_id, "voxel_positions", H5P_DEFAULT)) {
        hid_t dataset = H5Dopen2(group_id, "voxel_positions", H5P_DEFAULT);
        hid_t dataspace = H5Dget_space(dataset);
        
        hsize_t dims[2];
        H5Sget_simple_extent_dims(dataspace, dims, NULL);
        
        data.voxel_positions.resize(dims[0]);
        H5Dread(dataset, H5T_NATIVE_INT32, H5S_ALL, H5S_ALL, H5P_DEFAULT,
               data.voxel_positions.data());
        
        H5Sclose(dataspace);
        H5Dclose(dataset);
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
    
    // Write scalar values
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
    
    // Write bounding box
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
        // Statistics is optional
        return true;
    }
    
    hid_t group_id = H5Gopen2(file_id, "/statistics", H5P_DEFAULT);
    if (group_id < 0) {
        return false;
    }
    
    // Read scalar values
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
    
    // Read bounding box
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

} // namespace pointcloud_compressor
