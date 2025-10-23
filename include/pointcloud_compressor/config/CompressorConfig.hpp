// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_CONFIG_COMPRESSOR_CONFIG_HPP
#define POINTCLOUD_COMPRESSOR_CONFIG_COMPRESSOR_CONFIG_HPP

#include <string>

#include "pointcloud_compressor/core/PointCloudCompressor.hpp"

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
};

CompressorConfig loadCompressorConfigFromYaml(const std::string& path);

pointcloud_compressor::CompressionSettings settingsFromConfig(const CompressorConfig& config);

}  // namespace pointcloud_compressor::config

#endif  // POINTCLOUD_COMPRESSOR_CONFIG_COMPRESSOR_CONFIG_HPP
