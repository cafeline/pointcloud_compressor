// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_CONFIG_COMPRESSOR_CONFIG_HPP
#define POINTCLOUD_COMPRESSOR_CONFIG_COMPRESSOR_CONFIG_HPP

#include <string>
#include <vector>

#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/runtime/RuntimeAPI.hpp"

namespace pointcloud_compressor::config {

struct CompressorConfig {
    std::string input_file;
    double voxel_size = 0.01;
    int block_size = 8;
    int min_points_threshold = 1;
    bool publish_occupied_voxel_markers = false;
    bool save_hdf5 = false;
    std::string hdf5_output_file;
    bool save_raw_hdf5 = false;
    std::string raw_hdf5_output_file;

    std::vector<std::string> validate(bool check_filesystem = true) const;
};

struct BlockSizeOptimizationConfig {
    CompressorConfig base;
    int min_block_size = 4;
    int max_block_size = 32;
    int step_size = 1;
    bool auto_compress = false;
    bool verbose = false;

    std::vector<std::string> validate(bool check_filesystem = true) const;
};

CompressorConfig loadCompressorConfigFromYaml(const std::string& path);
BlockSizeOptimizationConfig loadBlockSizeOptimizationConfigFromYaml(const std::string& path);

pointcloud_compressor::CompressionSettings settingsFromConfig(const CompressorConfig& config);

}  // namespace pointcloud_compressor::config

#endif  // POINTCLOUD_COMPRESSOR_CONFIG_COMPRESSOR_CONFIG_HPP
