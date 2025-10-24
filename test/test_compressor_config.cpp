// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "pointcloud_compressor/config/CompressorConfig.hpp"
#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/runtime/RuntimeAPI.hpp"

using pointcloud_compressor::CompressionSettings;
using pointcloud_compressor::config::BlockSizeOptimizationConfig;
using pointcloud_compressor::config::CompressorConfig;
using pointcloud_compressor::config::loadBlockSizeOptimizationConfigFromYaml;
using pointcloud_compressor::config::loadCompressorConfigFromYaml;
using pointcloud_compressor::config::settingsFromConfig;

namespace {

std::string writeTempYaml(const std::string& contents) {
    auto temp_dir = std::filesystem::temp_directory_path();
    auto path = temp_dir / std::filesystem::path("pcc_config_test.yaml");
    std::ofstream ofs(path);
    ofs << contents;
    ofs.close();
    return path.string();
}

}  // namespace

TEST(CompressorConfigTest, LoadsValuesFromYaml) {
    const std::string yaml = R"(
pointcloud_compressor_node:
  ros__parameters:
    input_file: "/tmp/input.pcd"
    voxel_size: 0.2
    block_size: 12
    min_points_threshold: 5
    publish_occupied_voxel_markers: true
    save_hdf5: true
    hdf5_output_file: "/tmp/output.h5"
    save_raw_hdf5: true
    raw_hdf5_output_file: "/tmp/raw.h5"
)";

    const auto path = writeTempYaml(yaml);
    auto config = loadCompressorConfigFromYaml(path);

    EXPECT_EQ("/tmp/input.pcd", config.input_file);
    EXPECT_DOUBLE_EQ(0.2, config.voxel_size);
    EXPECT_EQ(12, config.block_size);
    EXPECT_EQ(5, config.min_points_threshold);
    EXPECT_TRUE(config.publish_occupied_voxel_markers);
    EXPECT_TRUE(config.save_hdf5);
    EXPECT_EQ("/tmp/output.h5", config.hdf5_output_file);
    EXPECT_TRUE(config.save_raw_hdf5);
    EXPECT_EQ("/tmp/raw.h5", config.raw_hdf5_output_file);

    CompressionSettings settings = settingsFromConfig(config);
    EXPECT_FLOAT_EQ(0.2f, settings.voxel_size);
    EXPECT_EQ(12, settings.block_size);
    EXPECT_EQ(5, settings.min_points_threshold);
}

TEST(CompressorConfigTest, AppliesDefaultsForMissingEntries) {
    const std::string yaml = R"(
ros__parameters:
  input_file: "/data/input.ply"
)";
    const auto path = writeTempYaml(yaml);
    auto config = loadCompressorConfigFromYaml(path);

    EXPECT_EQ("/data/input.ply", config.input_file);
    EXPECT_DOUBLE_EQ(0.01, config.voxel_size);
    EXPECT_EQ(8, config.block_size);
    EXPECT_EQ(1, config.min_points_threshold);
    EXPECT_FALSE(config.publish_occupied_voxel_markers);
    EXPECT_FALSE(config.save_hdf5);
    EXPECT_TRUE(config.hdf5_output_file.empty());

    auto settings = settingsFromConfig(config);
    EXPECT_FLOAT_EQ(0.01f, settings.voxel_size);
    EXPECT_EQ(8, settings.block_size);
    EXPECT_EQ(1, settings.min_points_threshold);
}

TEST(CompressorConfigTest, ValidateDetectsMissingInput) {
    CompressorConfig config;
    auto errors = config.validate(false);
    ASSERT_FALSE(errors.empty());
    EXPECT_NE(errors[0].find("input_file"), std::string::npos);
}

TEST(CompressorConfigTest, ValidateChecksOutputPaths) {
    const auto tmp_path = writeTempYaml("input");
    {
        std::ofstream ofs(tmp_path);
        ofs << "dummy";
    }
    CompressorConfig config;
    config.input_file = tmp_path;
    config.save_hdf5 = true;
    auto errors = config.validate(false);
    ASSERT_FALSE(errors.empty());
    EXPECT_NE(errors[0].find("hdf5"), std::string::npos);
}

TEST(CompressorConfigTest, BlockSizeOptimizationConfigDefaults) {
    BlockSizeOptimizationConfig opt;
    EXPECT_EQ(4, opt.min_block_size);
    EXPECT_EQ(32, opt.max_block_size);
    EXPECT_EQ(1, opt.step_size);
    EXPECT_FALSE(opt.auto_compress);
    EXPECT_FALSE(opt.verbose);
}

TEST(CompressorConfigTest, BlockSizeOptimizationConfigFromYaml) {
    const std::string yaml = R"(
pointcloud_compressor_node:
  ros__parameters:
    input_file: "/tmp/input.pcd"
    min_block_size: 6
    max_block_size: 18
    step_size: 2
    auto_compress: true
    verbose: true
)";
    const auto path = writeTempYaml(yaml);
    auto opt = loadBlockSizeOptimizationConfigFromYaml(path);

    EXPECT_EQ("/tmp/input.pcd", opt.base.input_file);
    EXPECT_EQ(6, opt.min_block_size);
    EXPECT_EQ(18, opt.max_block_size);
    EXPECT_EQ(2, opt.step_size);
    EXPECT_TRUE(opt.auto_compress);
    EXPECT_TRUE(opt.verbose);
}

TEST(CompressorConfigTest, BlockSizeOptimizationValidationChecksRanges) {
    const auto temp_input = writeTempYaml("input");
    BlockSizeOptimizationConfig opt;
    opt.base.input_file = temp_input;
    opt.step_size = 0;
    auto errors = opt.validate(false);
    ASSERT_FALSE(errors.empty());
    EXPECT_NE(errors[0].find("step_size"), std::string::npos);
}

TEST(CompressorConfigTest, ToCompressionRequestPopulatesFields) {
    CompressorConfig config;
    config.input_file = "/tmp/file.pcd";
    config.voxel_size = 0.05;
    config.block_size = 6;
    config.min_points_threshold = 3;
    config.save_hdf5 = true;
    config.hdf5_output_file = "/tmp/out.h5";
    config.save_raw_hdf5 = true;
    config.raw_hdf5_output_file = "/tmp/raw.h5";

    auto settings = settingsFromConfig(config);
    auto request = toCompressionRequest(config, settings);

    EXPECT_STREQ(config.input_file.c_str(), request.input_file);
    EXPECT_NEAR(config.voxel_size, request.voxel_size, 1e-6);
    EXPECT_EQ(config.block_size, request.block_size);
    EXPECT_TRUE(request.save_hdf5);
    EXPECT_STREQ(config.hdf5_output_file.c_str(), request.hdf5_output_path);
    EXPECT_TRUE(request.save_raw_hdf5);
    EXPECT_STREQ(config.raw_hdf5_output_file.c_str(), request.raw_hdf5_output_path);
}
