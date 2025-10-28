// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "vq_occupancy_compressor/config/CompressorConfig.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <stdexcept>

namespace vq_occupancy_compressor::config {

namespace {

YAML::Node extractParameterNode(const YAML::Node& root) {
    if (!root || root.IsNull()) {
        return YAML::Node();
    }

    if (root["block_size_optimizer"]) {
        auto node = root["block_size_optimizer"];
        if (node["ros__parameters"]) {
            return node["ros__parameters"];
        }
        return node;
    }

    if (root["vq_occupancy_compressor_node"]) {
        auto node = root["vq_occupancy_compressor_node"];
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

CompressorConfig parseCompressorConfig(const YAML::Node& params) {
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

}  

CompressorConfig loadCompressorConfigFromYaml(const std::string& path) {
    YAML::Node root = YAML::LoadFile(path);
    YAML::Node params = extractParameterNode(root);
    return parseCompressorConfig(params);
}

BlockSizeOptimizationConfig loadBlockSizeOptimizationConfigFromYaml(const std::string& path) {
    YAML::Node root = YAML::LoadFile(path);
    YAML::Node params = extractParameterNode(root);

    BlockSizeOptimizationConfig config;
    config.base = parseCompressorConfig(params);
    config.min_block_size = readOrDefault<int>(params, "min_block_size", config.min_block_size);
    config.max_block_size = readOrDefault<int>(params, "max_block_size", config.max_block_size);
    config.step_size = readOrDefault<int>(params, "step_size", config.step_size);
    config.verbose = readOrDefault<bool>(params, "verbose", config.verbose);
    return config;
}

std::vector<std::string> CompressorConfig::validate(bool check_filesystem) const {
    std::vector<std::string> errors;
    if (input_file.empty()) {
        errors.emplace_back("input_file is empty");
    } else if (check_filesystem && !std::filesystem::exists(input_file)) {
        errors.emplace_back("input_file does not exist: " + input_file);
    }

    if (voxel_size <= 0.0) {
        errors.emplace_back("voxel_size must be positive");
    }
    if (block_size <= 0) {
        errors.emplace_back("block_size must be greater than zero");
    }
    if (min_points_threshold < 0) {
        errors.emplace_back("min_points_threshold must be non-negative");
    }
    if (save_hdf5 && hdf5_output_file.empty()) {
        errors.emplace_back("hdf5_output_file must be set when save_hdf5 is true");
    }
    if (save_raw_hdf5 && raw_hdf5_output_file.empty()) {
        errors.emplace_back("raw_hdf5_output_file must be set when save_raw_hdf5 is true");
    }
    return errors;
}

std::vector<std::string> BlockSizeOptimizationConfig::validate(bool check_filesystem) const {
    std::vector<std::string> errors = base.validate(check_filesystem);
    if (min_block_size <= 0) {
        errors.emplace_back("min_block_size must be greater than zero");
    }
    if (max_block_size < min_block_size) {
        errors.emplace_back("max_block_size must be greater than or equal to min_block_size");
    }
    if (step_size <= 0) {
        errors.emplace_back("step_size must be greater than zero");
    }
    return errors;
}

CompressionSettings settingsFromConfig(const CompressorConfig& config) {
    CompressionSettings settings;
    settings.voxel_size = static_cast<float>(config.voxel_size);
    settings.block_size = config.block_size;
    settings.min_points_threshold = config.min_points_threshold;
    return settings;
}

}  
