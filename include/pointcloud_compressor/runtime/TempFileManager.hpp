// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_RUNTIME_TEMP_FILE_MANAGER_HPP
#define POINTCLOUD_COMPRESSOR_RUNTIME_TEMP_FILE_MANAGER_HPP

#include <string>

namespace pointcloud_compressor::runtime {

class TempFileManager {
public:
    TempFileManager() = default;
    ~TempFileManager() = default;

    std::string createTemporaryPrefix(const std::string& base_dir = "");
    void cleanupArtifacts(const std::string& prefix) const;
};

}  // namespace pointcloud_compressor::runtime

#endif  // POINTCLOUD_COMPRESSOR_RUNTIME_TEMP_FILE_MANAGER_HPP
