// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_compressor/runtime/RuntimeAPI.hpp"

#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/io/HDF5IO.hpp"
#include "pointcloud_compressor/runtime/CompressionArtifacts.hpp"
#include "pointcloud_compressor/runtime/CompressionReportBuilder.hpp"
#include "pointcloud_compressor/runtime/Hdf5Writers.hpp"
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

void maybeWriteCompressedMap(const CompressionResult& result,
                             const PCCCompressionRequest& request,
                             RuntimeHandle* impl,
                             const std::vector<uint8_t>& dictionary_buffer) {
    if (!request.save_hdf5 || request.hdf5_output_path == nullptr || request.hdf5_output_path[0] == '\0') {
        return;
    }

    const auto block_indices_u32 =
        pointcloud_compressor::runtime::convertBlockIndicesToU32(result.block_indices);

    pointcloud_compressor::runtime::CompressionReportBuilder builder;
    auto data = builder.toCompressedMapData(result, request, dictionary_buffer, block_indices_u32);

    std::string error;
    if (!pointcloud_compressor::runtime::writeCompressedMap(request.hdf5_output_path, data, error)) {
        if (impl) {
            if (!impl->error_message.empty()) {
                impl->error_message.append("; ");
            }
            impl->error_message.append(error);
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

    std::string error;
    if (!pointcloud_compressor::runtime::writeRawVoxelGrid(request.raw_hdf5_output_path, result, request, occupancy_buffer, error)) {
        if (impl) {
            if (!impl->error_message.empty()) {
                impl->error_message.append("; ");
            }
            impl->error_message.append(error);
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

    pointcloud_compressor::runtime::CompressionReportBuilder report_builder;
    auto report = report_builder.build(result, *request,
                                       impl->dictionary_buffer,
                                       impl->indices_buffer,
                                       impl->occupancy_buffer,
                                       impl->error_message);

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
