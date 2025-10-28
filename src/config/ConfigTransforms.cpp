// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "vq_occupancy_compressor/config/ConfigTransforms.hpp"

#include <vector>

namespace vq_occupancy_compressor::config {

PCCCompressionRequest toCompressionRequest(const CompressorConfig& config,
                                           const vq_occupancy_compressor::CompressionSettings& settings) {
    PCCCompressionRequest request{};
    request.input_file = config.input_file.c_str();
    request.voxel_size = static_cast<double>(settings.voxel_size);
    request.block_size = settings.block_size;
    request.min_points_threshold = settings.min_points_threshold;
    request.save_hdf5 = config.save_hdf5;
    request.hdf5_output_path = (config.save_hdf5 && !config.hdf5_output_file.empty())
                                   ? config.hdf5_output_file.c_str()
                                   : nullptr;
    request.save_raw_hdf5 = config.save_raw_hdf5;
    request.raw_hdf5_output_path = (config.save_raw_hdf5 && !config.raw_hdf5_output_file.empty())
                                       ? config.raw_hdf5_output_file.c_str()
                                       : nullptr;
    request.bounding_box_margin_ratio = settings.bounding_box_margin_ratio;
    return request;
}

CompressionSetup buildCompressionSetup(const CompressorConfig& config) {
    CompressionSetup setup;
    setup.config = config;
    setup.settings = settingsFromConfig(setup.config);
    setup.request = toCompressionRequest(setup.config, setup.settings);
    return setup;
}

std::vector<std::string> validateCompressionSetup(const CompressionSetup& setup) {
    return setup.config.validate(true);
}

}  
