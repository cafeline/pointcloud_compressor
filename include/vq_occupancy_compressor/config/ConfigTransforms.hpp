// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_CONFIG_CONFIG_TRANSFORMS_HPP
#define VQ_OCCUPANCY_COMPRESSOR_CONFIG_CONFIG_TRANSFORMS_HPP

#include <vector>

#include "vq_occupancy_compressor/config/CompressorConfig.hpp"

namespace vq_occupancy_compressor::config {

PCCCompressionRequest toCompressionRequest(const CompressorConfig& config,
                                           const vq_occupancy_compressor::CompressionSettings& settings);

struct CompressionSetup {
    CompressorConfig config;
    vq_occupancy_compressor::CompressionSettings settings;
    PCCCompressionRequest request;
};

CompressionSetup buildCompressionSetup(const CompressorConfig& config);

std::vector<std::string> validateCompressionSetup(const CompressionSetup& setup);

}  

#endif  
