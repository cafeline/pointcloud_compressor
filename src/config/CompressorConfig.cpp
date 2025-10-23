// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_compressor/config/CompressorConfig.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace pointcloud_compressor::config {

namespace {

YAML::Node extractParameterNode(const YAML::Node& root) {
    if (!root || root.IsNull()) {
        return YAML::Node();
    }

    if (root["pointcloud_compressor_node"]) {
        auto node = root["pointcloud_compressor_node"];
        if (node["ros__parameters"]) {
            return node["ros__parameters"];
        }
        return node;
    }

    if (root["ros__parameters"]) {
        return root["ros__parameters"];
    }

    return root;
}

template <typename T>
T readOrDefault(const YAML::Node& node, const std::string& key, const T& default_value) {
    if (!node || !node[key]) {
        return default_value;
    }
    return node[key].as<T>();
}

}  // namespace

CompressorConfig loadCompressorConfigFromYaml(const std::string& path) {
    YAML::Node root = YAML::LoadFile(path);
    YAML::Node params = extractParameterNode(root);

    CompressorConfig config;
    config.input_file = readOrDefault<std::string>(params, "input_file", "");
    config.voxel_size = readOrDefault<double>(params, "voxel_size", 0.01);
    config.block_size = readOrDefault<int>(params, "block_size", 8);
    config.min_points_threshold = readOrDefault<int>(params, "min_points_threshold", 1);
    config.publish_occupied_voxel_markers =
        readOrDefault<bool>(params, "publish_occupied_voxel_markers", false);
    config.save_hdf5 = readOrDefault<bool>(params, "save_hdf5", false);
    config.hdf5_output_file = readOrDefault<std::string>(params, "hdf5_output_file", "");
    config.save_raw_hdf5 = readOrDefault<bool>(params, "save_raw_hdf5", false);
    config.raw_hdf5_output_file =
        readOrDefault<std::string>(params, "raw_hdf5_output_file", "");

    return config;
}

CompressionSettings settingsFromConfig(const CompressorConfig& config) {
    CompressionSettings settings;
    settings.voxel_size = static_cast<float>(config.voxel_size);
    settings.block_size = config.block_size;
    settings.min_points_threshold = config.min_points_threshold;
    return settings;
}

}  // namespace pointcloud_compressor::config
