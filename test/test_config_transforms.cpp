// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "pointcloud_compressor/config/ConfigTransforms.hpp"

using pointcloud_compressor::config::buildCompressionSetup;
using pointcloud_compressor::config::validateForRuntime;
using pointcloud_compressor::config::CompressorConfig;

namespace {

std::string createTempInputFile() {
    auto path = std::filesystem::temp_directory_path() / "pcc_config_setup_input.pcd";
    std::ofstream ofs(path);
    ofs << "dummy";
    return path.string();
}

}  // namespace

TEST(ConfigTransformsTest, BuildSetupProvidesStablePointers) {
    CompressorConfig config;
    config.input_file = createTempInputFile();
    config.hdf5_output_file = "/tmp/out.h5";
    config.save_hdf5 = true;
    config.raw_hdf5_output_file = "/tmp/raw.h5";
    config.save_raw_hdf5 = true;

    auto setup = buildCompressionSetup(config);
    EXPECT_STREQ(config.input_file.c_str(), setup.request.input_file);
    EXPECT_STREQ(config.hdf5_output_file.c_str(), setup.request.hdf5_output_path);
    EXPECT_STREQ(config.raw_hdf5_output_file.c_str(), setup.request.raw_hdf5_output_path);
}

TEST(ConfigTransformsTest, ValidateForRuntimeDetectsMissingPaths) {
    CompressorConfig config;
    config.input_file = "/tmp/does_not_exist.pcd";
    config.save_hdf5 = true;

    auto setup = buildCompressionSetup(config);
    auto errors = validateForRuntime(setup);
    ASSERT_FALSE(errors.empty());
    EXPECT_NE(errors.front().find("input_file"), std::string::npos);
}
