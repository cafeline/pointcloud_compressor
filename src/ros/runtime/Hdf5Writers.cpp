// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_compressor/runtime/Hdf5Writers.hpp"

#include <filesystem>

#include "pointcloud_compressor/io/HDF5IO.hpp"
#include "pointcloud_compressor/runtime/CompressionReportBuilder.hpp"

namespace pointcloud_compressor::runtime {

namespace {

HDF5IO::RawVoxelGridData buildRawGridData(const CompressionResult& result,
                                          const PCCCompressionRequest& request,
                                          const std::vector<uint8_t>& occupancy_buffer) {
    HDF5IO::RawVoxelGridData raw;
    const auto dims = result.voxel_grid.getDimensions();
    raw.dim_x = static_cast<uint32_t>(dims.x);
    raw.dim_y = static_cast<uint32_t>(dims.y);
    raw.dim_z = static_cast<uint32_t>(dims.z);
    raw.voxel_size = static_cast<float>(request.voxel_size);

    float origin_x = 0.0f;
    float origin_y = 0.0f;
    float origin_z = 0.0f;
    result.voxel_grid.getOrigin(origin_x, origin_y, origin_z);
    raw.origin = {origin_x, origin_y, origin_z};

    raw.voxel_values.clear();
    raw.voxel_values.reserve(occupancy_buffer.size());
    for (uint8_t value : occupancy_buffer) {
        raw.voxel_values.push_back(value ? 255u : 0u);
    }

    raw.occupied_voxels.clear();
    raw.occupied_voxels.reserve(result.voxel_grid.getOccupiedVoxelCount());

    std::size_t index = 0;
    for (uint32_t z = 0; z < raw.dim_z; ++z) {
        for (uint32_t y = 0; y < raw.dim_y; ++y) {
            for (uint32_t x = 0; x < raw.dim_x; ++x, ++index) {
                if (index < occupancy_buffer.size() && occupancy_buffer[index]) {
                    raw.occupied_voxels.push_back(
                        {static_cast<int32_t>(x), static_cast<int32_t>(y), static_cast<int32_t>(z)});
                }
            }
        }
    }

    return raw;
}

}  // namespace

bool writeCompressedMap(const std::string& output_path,
                        const pointcloud_compressor::CompressedMapData& data,
                        std::string& error_message) {
    if (output_path.empty()) {
        return false;
    }

    HDF5IO hdf5_io;
    if (!hdf5_io.write(output_path, data)) {
        if (!error_message.empty()) {
            error_message.append("; ");
        }
        error_message.append("HDF5 write failed: ");
        error_message.append(hdf5_io.getLastError());
        return false;
    }
    return true;
}

bool writeRawVoxelGrid(const std::string& output_path,
                       const CompressionResult& result,
                       const PCCCompressionRequest& request,
                       const std::vector<uint8_t>& occupancy_buffer,
                       std::string& error_message) {
    if (output_path.empty()) {
        return false;
    }

    HDF5IO hdf5_io;
    auto raw = buildRawGridData(result, request, occupancy_buffer);
    if (!hdf5_io.writeRawVoxelGrid(output_path, raw)) {
        if (!error_message.empty()) {
            error_message.append("; ");
        }
        error_message.append("Raw HDF5 write failed: ");
        error_message.append(hdf5_io.getLastError());
        return false;
    }
    return true;
}

bool writeRawVoxelGrid(const std::string& output_path,
                       const PCCCompressionReport& report,
                       std::string& error_message) {
    if (output_path.empty()) {
        return false;
    }

    if (!report.occupancy.occupancy || report.occupancy.size == 0) {
        return false;
    }

    HDF5IO hdf5_io;
    HDF5IO::RawVoxelGridData raw;
    raw.dim_x = report.occupancy.dimensions[0];
    raw.dim_y = report.occupancy.dimensions[1];
    raw.dim_z = report.occupancy.dimensions[2];
    raw.voxel_size = static_cast<float>(report.grid.voxel_size);
    raw.origin = {report.occupancy.origin[0], report.occupancy.origin[1], report.occupancy.origin[2]};
    raw.voxel_values.reserve(report.occupancy.size);
    for (std::size_t i = 0; i < report.occupancy.size; ++i) {
        raw.voxel_values.push_back(report.occupancy.occupancy[i] ? 255u : 0u);
    }

    for (uint32_t z = 0; z < raw.dim_z; ++z) {
        for (uint32_t y = 0; y < raw.dim_y; ++y) {
            for (uint32_t x = 0; x < raw.dim_x; ++x) {
                const std::size_t index = static_cast<std::size_t>(z) * raw.dim_y * raw.dim_x +
                                          static_cast<std::size_t>(y) * raw.dim_x +
                                          static_cast<std::size_t>(x);
                if (index < raw.voxel_values.size() && raw.voxel_values[index]) {
                    raw.occupied_voxels.push_back({static_cast<int32_t>(x),
                                                   static_cast<int32_t>(y),
                                                   static_cast<int32_t>(z)});
                }
            }
        }
    }

    if (!hdf5_io.writeRawVoxelGrid(output_path, raw)) {
        if (!error_message.empty()) {
            error_message.append("; ");
        }
        error_message.append("Raw HDF5 write failed: ");
        error_message.append(hdf5_io.getLastError());
        return false;
    }
    return true;
}

}  // namespace pointcloud_compressor::runtime
