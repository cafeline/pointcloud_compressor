// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_compressor/runtime/TempFileManager.hpp"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <string>
#include <vector>

namespace pointcloud_compressor::runtime {

namespace {
constexpr const char* kArtifacts[] = {"_dict.bin", "_indices.bin", "_meta.bin"};
}

std::string TempFileManager::createTemporaryPrefix(const std::string& base_dir) {
    const std::filesystem::path root = base_dir.empty()
        ? std::filesystem::temp_directory_path()
        : std::filesystem::path(base_dir);

    static std::atomic<uint64_t> counter{0};
    const auto timestamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const uint64_t unique_id = counter.fetch_add(1, std::memory_order_relaxed);
    const std::string suffix = "pcc_runtime_" + std::to_string(timestamp) + "_" + std::to_string(unique_id);
    return (root / suffix).string();
}

void TempFileManager::cleanupArtifacts(const std::string& prefix) const {
    if (prefix.empty()) {
        return;
    }

    for (const auto* extension : kArtifacts) {
        std::filesystem::path path(prefix + extension);
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
}

}  // namespace pointcloud_compressor::runtime
