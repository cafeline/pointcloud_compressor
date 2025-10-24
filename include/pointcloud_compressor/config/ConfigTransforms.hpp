// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_CONFIG_CONFIG_TRANSFORMS_HPP
#define POINTCLOUD_COMPRESSOR_CONFIG_CONFIG_TRANSFORMS_HPP

#include "pointcloud_compressor/config/CompressorConfig.hpp"

namespace pointcloud_compressor::config {

PCCCompressionRequest toCompressionRequest(const CompressorConfig& config,
                                           const pointcloud_compressor::CompressionSettings& settings);

struct CompressionSetup {
    CompressorConfig config;
    pointcloud_compressor::CompressionSettings settings;
    PCCCompressionRequest request;
};

CompressionSetup buildCompressionSetup(const CompressorConfig& config);

std::vector<std::string> validateForRuntime(const CompressionSetup& setup);

}  // namespace pointcloud_compressor::config

#endif  // POINTCLOUD_COMPRESSOR_CONFIG_CONFIG_TRANSFORMS_HPP
