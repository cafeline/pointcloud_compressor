// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "vq_occupancy_compressor/services/CompressionExecutor.hpp"

#include <filesystem>
#include <stdexcept>

#include "vq_occupancy_compressor/common/CompressionArtifacts.hpp"
#include "vq_occupancy_compressor/common/CompressionDataUtils.hpp"

namespace vq_occupancy_compressor::services {

CompressionExecutor::CompressionExecutor()
    : compressor_(vq_occupancy_compressor::CompressionSettings()) {
    clearBuffers();
    last_report_ = makeEmptyReport();
}

PCCCompressionReport CompressionExecutor::compress(const PCCCompressionRequest& request) {
    clearBuffers();

    vq_occupancy_compressor::CompressionSettings settings;
    settings.voxel_size = static_cast<float>(request.voxel_size);
    settings.block_size = request.block_size;
    settings.min_points_threshold = request.min_points_threshold;
    settings.bounding_box_margin_ratio = static_cast<float>(request.bounding_box_margin_ratio);

    compressor_.updateSettings(settings);

    vq_occupancy_compressor::CompressionResult result;
    try {
        result = compressor_.compress(request.input_file);
    } catch (const std::exception& e) {
        return makeErrorReport(std::string("Compression threw exception: ") + e.what());
    } catch (...) {
        return makeErrorReport("Compression threw unknown exception");
    }

    if (!result.success) {
        return makeErrorReport(result.error_message.empty() ? "Compression failed" : result.error_message);
    }

    auto report = report_builder_.build(result,
                                        request,
                                        dictionary_buffer_,
                                        indices_buffer_,
                                        occupancy_buffer_,
                                        error_message_);
    last_report_ = report;

    const auto block_indices_u32 = vq_occupancy_compressor::common::convertBlockIndicesToU32(result.block_indices);
    map_data_ = report_builder_.toCompressedMapData(result,
                                                    request,
                                                    dictionary_buffer_,
                                                    block_indices_u32);

    vq_occupancy_compressor::utils::ErrorAccumulator error_acc;
    if (request.save_hdf5 && request.hdf5_output_path && request.hdf5_output_path[0] != '\0') {
        std::string err;
        if (!vq_occupancy_compressor::io::writeCompressedMap(request.hdf5_output_path, map_data_, err)) {
            error_acc.add(err);
        }
    }

    if (request.save_raw_hdf5 && request.raw_hdf5_output_path && request.raw_hdf5_output_path[0] != '\0') {
        std::string raw_errors;
        if (!vq_occupancy_compressor::io::writeRawVoxelGrid(request.raw_hdf5_output_path,
                                                          result,
                                                          request,
                                                          occupancy_buffer_,
                                                          raw_errors)) {
            error_acc.add(raw_errors);
        }
    }

    if (!error_acc.empty()) {
        if (!error_message_.empty()) {
            error_message_.append("; ");
        }
        error_message_.append(error_acc.str());
    }

    last_report_.error_message = error_message_.empty() ? nullptr : error_message_.c_str();
    return last_report_;
}

void CompressionExecutor::release(PCCCompressionReport& report) {
    clearBuffers();
    report = makeEmptyReport();
    last_report_ = report;
}

PCCCompressionReport CompressionExecutor::makeEmptyReport() const {
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

PCCCompressionReport CompressionExecutor::makeErrorReport(const std::string& message) {
    clearBuffers();
    error_message_ = message;
    last_report_ = makeEmptyReport();
    last_report_.error_message = error_message_.c_str();
    return last_report_;
}

void CompressionExecutor::clearBuffers() {
    dictionary_buffer_.clear();
    indices_buffer_.clear();
    occupancy_buffer_.clear();
    error_message_.clear();
    map_data_ = vq_occupancy_compressor::CompressedMapData{};
}

bool runCompression(const vq_occupancy_compressor::config::CompressionSetup& setup,
                    const CompressionSuccessCallback& on_success,
                    std::string* error_message) {
    if (!on_success) {
        if (error_message) {
            *error_message = "Compression callback is not set";
        }
        return false;
    }

    CompressionExecutor executor;
    auto report = executor.compress(setup.request);

    auto releaseReport = [&]() {
        executor.release(report);
    };

    if (!report.success) {
        if (error_message) {
            *error_message = report.error_message ? report.error_message : "Unknown error";
        }
        releaseReport();
        return false;
    }

    vq_occupancy_compressor::io::CompressionReportBuilder builder;
    try {
        on_success(report, builder);
    } catch (...) {
        releaseReport();
        throw;
    }

    releaseReport();
    return true;
}

}  
