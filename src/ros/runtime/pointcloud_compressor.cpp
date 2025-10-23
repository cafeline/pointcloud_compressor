// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_compressor/runtime/RuntimeAPI.hpp"

#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/io/HDF5IO.hpp"
#include "pointcloud_compressor/runtime/CompressionArtifacts.hpp"
#include "pointcloud_compressor/runtime/RuntimeHelpers.hpp"
#include "pointcloud_compressor/runtime/TempFileManager.hpp"

#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

using pointcloud_compressor::CompressionResult;
using pointcloud_compressor::CompressionSettings;
using pointcloud_compressor::HDF5IO;
using pointcloud_compressor::PointCloudCompressor;
using pointcloud_compressor::VoxelGrid;

struct RuntimeHandle {
    std::unique_ptr<PointCloudCompressor> compressor;
    std::vector<uint8_t> dictionary_buffer;
    std::vector<uint8_t> indices_buffer;
    std::vector<uint8_t> occupancy_buffer;
    std::string error_message;
};

RuntimeHandle* toImpl(PCCRuntimeHandle* handle) {
    return reinterpret_cast<RuntimeHandle*>(handle);
}

PCCCompressionReport makeEmptyReport() {
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

PCCCompressionReport makeErrorReport(RuntimeHandle* impl, const std::string& message) {
    PCCCompressionReport report = makeEmptyReport();
    if (impl) {
        impl->error_message = message;
        report.error_message = impl->error_message.c_str();
    }
    return report;
}

void populateRawHdf5Data(const CompressionResult& result,
                         const PCCCompressionRequest& request,
                         HDF5IO::RawVoxelGridData& raw_data,
                         std::vector<uint8_t>& occupancy_buffer) {
    const auto dims = result.voxel_grid.getDimensions();
    raw_data.dim_x = static_cast<uint32_t>(dims.x);
    raw_data.dim_y = static_cast<uint32_t>(dims.y);
    raw_data.dim_z = static_cast<uint32_t>(dims.z);
    raw_data.voxel_size = static_cast<float>(request.voxel_size);

    float origin_x = 0.0f;
    float origin_y = 0.0f;
    float origin_z = 0.0f;
    result.voxel_grid.getOrigin(origin_x, origin_y, origin_z);
    raw_data.origin = {origin_x, origin_y, origin_z};

    raw_data.voxel_values.clear();
    raw_data.voxel_values.reserve(occupancy_buffer.size());
    for (uint8_t value : occupancy_buffer) {
        raw_data.voxel_values.push_back(value ? 255u : 0u);
    }

    raw_data.occupied_voxels.clear();
    raw_data.occupied_voxels.reserve(result.voxel_grid.getOccupiedVoxelCount());

    std::size_t index = 0;
    for (uint32_t z = 0; z < raw_data.dim_z; ++z) {
        for (uint32_t y = 0; y < raw_data.dim_y; ++y) {
            for (uint32_t x = 0; x < raw_data.dim_x; ++x, ++index) {
                if (index < occupancy_buffer.size() && occupancy_buffer[index]) {
                    raw_data.occupied_voxels.push_back(
                        {static_cast<int32_t>(x),
                         static_cast<int32_t>(y),
                         static_cast<int32_t>(z)});
                }
            }
        }
    }
}

PCCCompressionReport buildSuccessReport(RuntimeHandle* impl,
                                        const CompressionResult& result,
                                        const PCCCompressionRequest& request) {
    PCCCompressionReport report = makeEmptyReport();
    report.success = true;

    impl->dictionary_buffer = pointcloud_compressor::runtime::flattenDictionaryPatterns(result.pattern_dictionary);
    impl->indices_buffer = pointcloud_compressor::runtime::packBlockIndices(
        result.block_indices, static_cast<uint8_t>(result.index_bit_size));
    impl->occupancy_buffer = pointcloud_compressor::runtime::buildOccupancyMask(result.voxel_grid);

    report.dictionary.num_patterns = static_cast<uint32_t>(result.pattern_dictionary.size());
    const uint32_t pattern_bits = static_cast<uint32_t>(request.block_size) *
                                  static_cast<uint32_t>(request.block_size) *
                                  static_cast<uint32_t>(request.block_size);
    report.dictionary.pattern_size_bytes = (pattern_bits + 7) / 8;
    report.dictionary.size = impl->dictionary_buffer.size();
    report.dictionary.data = impl->dictionary_buffer.empty() ? nullptr : impl->dictionary_buffer.data();

    report.indices.data = impl->indices_buffer.empty() ? nullptr : impl->indices_buffer.data();
    report.indices.size = impl->indices_buffer.size();
    report.indices.index_bit_size = static_cast<uint8_t>(result.index_bit_size);
    report.indices.total_blocks = static_cast<uint32_t>(result.num_blocks);

    report.grid.dimensions[0] = result.grid_dimensions.x;
    report.grid.dimensions[1] = result.grid_dimensions.y;
    report.grid.dimensions[2] = result.grid_dimensions.z;
    report.grid.origin[0] = result.grid_origin.x;
    report.grid.origin[1] = result.grid_origin.y;
    report.grid.origin[2] = result.grid_origin.z;
    report.grid.blocks_per_axis[0] = static_cast<uint32_t>(result.blocks_count.x);
    report.grid.blocks_per_axis[1] = static_cast<uint32_t>(result.blocks_count.y);
    report.grid.blocks_per_axis[2] = static_cast<uint32_t>(result.blocks_count.z);
    report.grid.voxel_size = request.voxel_size;

    report.occupancy.occupancy = impl->occupancy_buffer.empty() ? nullptr : impl->occupancy_buffer.data();
    report.occupancy.size = impl->occupancy_buffer.size();
    report.occupancy.dimensions[0] = static_cast<uint32_t>(result.voxel_grid.getDimensions().x);
    report.occupancy.dimensions[1] = static_cast<uint32_t>(result.voxel_grid.getDimensions().y);
    report.occupancy.dimensions[2] = static_cast<uint32_t>(result.voxel_grid.getDimensions().z);
    float origin_x = 0.0f;
    float origin_y = 0.0f;
    float origin_z = 0.0f;
    result.voxel_grid.getOrigin(origin_x, origin_y, origin_z);
    report.occupancy.origin[0] = origin_x;
    report.occupancy.origin[1] = origin_y;
    report.occupancy.origin[2] = origin_z;

    report.statistics.compression_ratio = result.compression_ratio;
    report.statistics.original_point_count = static_cast<uint32_t>(result.point_count);
    report.statistics.compressed_data_size = static_cast<uint32_t>(result.compressed_size);

    report.max_index = result.max_index;
    report.error_message = nullptr;
    impl->error_message.clear();
    return report;
}

void maybeWriteCompressedMap(const CompressionResult& result,
                             const PCCCompressionRequest& request,
                             RuntimeHandle* impl,
                             const std::vector<uint8_t>& dictionary_buffer) {
    if (!request.save_hdf5 || request.hdf5_output_path == nullptr || request.hdf5_output_path[0] == '\0') {
        return;
    }

    const auto block_indices_u32 =
        pointcloud_compressor::runtime::convertBlockIndicesToU32(result.block_indices);

    pointcloud_compressor::CompressedMapData data;
    pointcloud_compressor::runtime::populateCompressedMapData(
        result, request, dictionary_buffer, block_indices_u32, data);

    HDF5IO hdf5_io;
    if (!hdf5_io.write(request.hdf5_output_path, data)) {
        if (impl) {
            if (!impl->error_message.empty()) {
                impl->error_message.append("; ");
            }
            impl->error_message.append("HDF5 write failed: ");
            impl->error_message.append(hdf5_io.getLastError());
        }
    }
}

void maybeWriteRawGrid(const CompressionResult& result,
                       const PCCCompressionRequest& request,
                       RuntimeHandle* impl,
                       std::vector<uint8_t>& occupancy_buffer) {
    if (!request.save_raw_hdf5 || request.raw_hdf5_output_path == nullptr ||
        request.raw_hdf5_output_path[0] == '\0') {
        return;
    }

    HDF5IO hdf5_io;
    HDF5IO::RawVoxelGridData raw;
    populateRawHdf5Data(result, request, raw, occupancy_buffer);
    if (!hdf5_io.writeRawVoxelGrid(request.raw_hdf5_output_path, raw)) {
        if (impl) {
            if (!impl->error_message.empty()) {
                impl->error_message.append("; ");
            }
            impl->error_message.append("Raw HDF5 write failed: ");
            impl->error_message.append(hdf5_io.getLastError());
        }
    }
}

}  // namespace

extern "C" PCCRuntimeHandle* pcc_runtime_create() {
    auto impl = std::make_unique<RuntimeHandle>();
    if (!impl) {
        return nullptr;
    }

    impl->compressor = std::make_unique<PointCloudCompressor>();
    if (!impl->compressor) {
        return nullptr;
    }

    return reinterpret_cast<PCCRuntimeHandle*>(impl.release());
}

extern "C" void pcc_runtime_destroy(PCCRuntimeHandle* handle) {
    if (!handle) {
        return;
    }
    auto* impl = toImpl(handle);
    delete impl;
}

extern "C" PCCCompressionReport pcc_runtime_compress(PCCRuntimeHandle* handle,
                                                     const PCCCompressionRequest* request) {
    auto* impl = toImpl(handle);
    if (!impl) {
        return makeErrorReport(nullptr, "Invalid runtime handle");
    }

    if (!request) {
        return makeErrorReport(impl, "Compression request is null");
    }
    if (!request->input_file || request->input_file[0] == '\0') {
        return makeErrorReport(impl, "Input file path is empty");
    }
    if (request->voxel_size <= 0.0) {
        return makeErrorReport(impl, "Voxel size must be positive");
    }
    if (request->block_size <= 0) {
        return makeErrorReport(impl, "Block size must be greater than zero");
    }
    if (!impl->compressor) {
        return makeErrorReport(impl, "Compressor is not initialized");
    }

    impl->dictionary_buffer.clear();
    impl->indices_buffer.clear();
    impl->occupancy_buffer.clear();
    impl->error_message.clear();

    CompressionSettings settings;
    settings.voxel_size = static_cast<float>(request->voxel_size);
    settings.block_size = request->block_size;
    settings.use_8bit_indices = request->use_8bit_indices;
    settings.index_bit_size = request->use_8bit_indices ? 8 : 0;
    settings.min_points_threshold = request->min_points_threshold;
    settings.bounding_box_margin_ratio = static_cast<float>(request->bounding_box_margin_ratio);

    const auto temp_root = std::filesystem::temp_directory_path().string();
    settings.temp_directory = temp_root;
    impl->compressor->updateSettings(settings);

    pointcloud_compressor::runtime::TempFileManager temp_manager;
    const auto temp_prefix = temp_manager.createTemporaryPrefix(temp_root);

    struct CleanupGuard {
        pointcloud_compressor::runtime::TempFileManager* manager;
        std::string prefix;
        ~CleanupGuard() {
            if (manager) {
                manager->cleanupArtifacts(prefix);
            }
        }
    } cleanup_guard{&temp_manager, temp_prefix};

    CompressionResult result;
    try {
        result = impl->compressor->compress(request->input_file, temp_prefix);
    } catch (const std::exception& e) {
        return makeErrorReport(impl, std::string("Compression threw exception: ") + e.what());
    } catch (...) {
        return makeErrorReport(impl, "Compression threw unknown exception");
    }

    if (!result.success) {
        return makeErrorReport(impl, result.error_message.empty()
                                     ? "Compression failed"
                                     : result.error_message);
    }

    auto report = buildSuccessReport(impl, result, *request);

    maybeWriteCompressedMap(result, *request, impl, impl->dictionary_buffer);
    maybeWriteRawGrid(result, *request, impl, impl->occupancy_buffer);

    if (!impl->error_message.empty()) {
        report.error_message = impl->error_message.c_str();
    }

    return report;
}

extern "C" void pcc_runtime_release_report(PCCRuntimeHandle* handle, PCCCompressionReport* report) {
    if (!handle || !report) {
        return;
    }
    auto* impl = toImpl(handle);
    impl->dictionary_buffer.clear();
    impl->indices_buffer.clear();
    impl->occupancy_buffer.clear();
    impl->error_message.clear();
    *report = makeEmptyReport();
}
