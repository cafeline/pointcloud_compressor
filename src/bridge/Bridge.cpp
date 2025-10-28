// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "vq_occupancy_compressor/bridge/Bridge.hpp"

#include "vq_occupancy_compressor/services/CompressionExecutor.hpp"

#include <memory>
#include <string>

namespace {

struct CompressionHandleImpl {
    std::unique_ptr<vq_occupancy_compressor::services::CompressionExecutor> service;
    std::string last_error;
};

CompressionHandleImpl* toImpl(VqoCompressionHandle* handle) {
    return reinterpret_cast<CompressionHandleImpl*>(handle);
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

PCCCompressionReport makeErrorReport(CompressionHandleImpl* impl, const std::string& message) {
    PCCCompressionReport report = makeEmptyReport();
    if (impl) {
        impl->last_error = message;
        report.error_message = impl->last_error.c_str();
    }
    return report;
}

}  

extern "C" VqoCompressionHandle* vqo_handle_create() {
    auto impl = std::make_unique<CompressionHandleImpl>();
    if (!impl) {
        return nullptr;
    }

    impl->service = std::make_unique<vq_occupancy_compressor::services::CompressionExecutor>();
    if (!impl->service) {
        return nullptr;
    }

    return reinterpret_cast<VqoCompressionHandle*>(impl.release());
}

extern "C" void vqo_handle_destroy(VqoCompressionHandle* handle) {
    if (!handle) {
        return;
    }
    auto* impl = toImpl(handle);
    delete impl;
}

extern "C" PCCCompressionReport vqo_handle_compress(VqoCompressionHandle* handle,
                                                    const PCCCompressionRequest* request) {
    auto* impl = toImpl(handle);
    if (!impl) {
        return makeErrorReport(nullptr, "Invalid compression handle");
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
    try {
        return impl->service->compress(*request);
    } catch (const std::exception& e) {
        return makeErrorReport(impl, std::string("Compression threw exception: ") + e.what());
    } catch (...) {
        return makeErrorReport(impl, "Compression threw unknown exception");
    }
}

extern "C" void vqo_handle_release_report(VqoCompressionHandle* handle, PCCCompressionReport* report) {
    if (!handle || !report) {
        return;
    }
    auto* impl = toImpl(handle);
    impl->service->release(*report);
    *report = makeEmptyReport();
}
