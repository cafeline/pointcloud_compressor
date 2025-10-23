// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "pointcloud_compressor/config/CompressorConfig.hpp"
#include "pointcloud_compressor/core/PointCloudCompressor.hpp"

using pointcloud_compressor::CompressionSettings;
using pointcloud_compressor::config::CompressorConfig;
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
