// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "pointcloud_compressor/runtime/TempFileManager.hpp"

using pointcloud_compressor::runtime::TempFileManager;

TEST(TempFileManagerTest, GeneratesDistinctPrefixes) {
    TempFileManager manager;
    const auto prefix1 = manager.createTemporaryPrefix();
    const auto prefix2 = manager.createTemporaryPrefix();
    EXPECT_FALSE(prefix1.empty());
    EXPECT_FALSE(prefix2.empty());
    EXPECT_NE(prefix1, prefix2);
}

TEST(TempFileManagerTest, CleanupRemovesArtifacts) {
    TempFileManager manager;
    const auto prefix = manager.createTemporaryPrefix();

    const std::vector<std::string> filenames = {
        prefix + "_dict.bin",
        prefix + "_indices.bin",
        prefix + "_meta.bin"
    };
    for (const auto& file : filenames) {
        std::ofstream ofs(file);
        ASSERT_TRUE(ofs.good());
        ofs << "temp";
    }

    manager.cleanupArtifacts(prefix);

    for (const auto& file : filenames) {
        EXPECT_FALSE(std::filesystem::exists(file));
    }
}
