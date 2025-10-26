// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_compressor/config/CliConfigParser.hpp"
#include "pointcloud_compressor/config/ConfigTransforms.hpp"
#include "pointcloud_compressor/config/CompressorConfig.hpp"
#include "pointcloud_compressor/core/BlockSizeReportFormatter.hpp"
#include "pointcloud_compressor/core/CompressionReportFormatter.hpp"
#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/io/Hdf5Writers.hpp"
#include "pointcloud_compressor/utils/ErrorAccumulator.hpp"
#include "pointcloud_compressor/services/CompressionExecutor.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {

using pointcloud_compressor::PointCloudCompressor;
using pointcloud_compressor::CompressionSettings;
using pointcloud_compressor::config::BlockSizeOptimizationConfig;
using pointcloud_compressor::config::CompressionSetup;
using pointcloud_compressor::config::CompressorConfig;
using pointcloud_compressor::config::buildCompressionSetup;
using pointcloud_compressor::config::loadBlockSizeOptimizationConfigFromYaml;
using pointcloud_compressor::config::loadCompressorConfigFromYaml;
using pointcloud_compressor::config::parseConfigPath;
using pointcloud_compressor::config::validateCompressionSetup;
using pointcloud_compressor::io::CompressionReportBuilder;
using pointcloud_compressor::services::runCompression;

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " <command> [options]\n";
    std::cout << "\nCommands:\n";
    std::cout << "  compress <config.yaml>                    - Compress using YAML configuration\n";
    std::cout << "  optimize <config.yaml>                    - Run block size optimization using YAML\n";
}

void printCompressionSummary(const PCCCompressionReport& report, const std::string& output_h5) {
    std::cout << pointcloud_compressor::formatCompressionSummary(report) << "\n";
    std::cout << "  Output archive   : " << output_h5 << "\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    const std::string command = argv[1];

    try {
        if (command == "compress") {
            std::vector<std::string> args(argv + 2, argv + argc);
            std::string config_path;
            try {
                config_path = parseConfigPath(args);
            } catch (const std::exception& e) {
                std::cerr << e.what() << "\n";
                printUsage(argv[0]);
                return 1;
            }

            CompressionSetup setup;
            try {
                setup = buildCompressionSetup(loadCompressorConfigFromYaml(config_path));
            } catch (const std::exception& e) {
                std::cerr << "Failed to load config: " << e.what() << "\n";
                return 1;
            }

            auto errors = validateCompressionSetup(setup);
            if (!errors.empty()) {
                for (const auto& err : errors) {
                    std::cerr << "Config error: " << err << "\n";
                }
                return 1;
            }

            pointcloud_compressor::utils::ErrorAccumulator error_acc;
            std::string compression_error;
            const bool compression_success = runCompression(
                setup,
                [&](const PCCCompressionReport& report,
                    pointcloud_compressor::io::CompressionReportBuilder& builder) {
                    auto map_data = builder.toCompressedMapData(report,
                                                                setup.settings.voxel_size,
                                                                setup.settings.block_size);

                    if (setup.request.save_hdf5 && setup.request.hdf5_output_path) {
                        std::string err;
                        if (!pointcloud_compressor::io::writeCompressedMap(setup.config.hdf5_output_file,
                                                                           map_data,
                                                                           err)) {
                            error_acc.add(err);
                        } else {
                            printCompressionSummary(report, setup.config.hdf5_output_file);
                        }
                    } else {
                        std::cout << "Compression successful. No archive was saved (save_hdf5=false).\n";
                    }

                    if (setup.request.save_raw_hdf5 && setup.request.raw_hdf5_output_path) {
                        std::string err;
                        if (pointcloud_compressor::io::writeRawVoxelGrid(setup.config.raw_hdf5_output_file,
                                                                         report,
                                                                         err)) {
                            std::cout << "  Raw voxel grid   : " << setup.config.raw_hdf5_output_file << "\n";
                        } else {
                            error_acc.add(err);
                        }
                    }
                },
                &compression_error);

            if (!compression_success) {
                std::cerr << "Compression failed: " << compression_error << "\n";
                return 1;
            }

            if (!error_acc.empty()) {
                std::cerr << error_acc.str() << "\n";
            }

            std::cout << "Processing completed!!\n";

        } else if (command == "optimize") {
            std::vector<std::string> args(argv + 2, argv + argc);
            std::string config_path;
            try {
                config_path = parseConfigPath(args);
            } catch (const std::exception& e) {
                std::cerr << e.what() << "\n";
                printUsage(argv[0]);
                return 1;
            }

            BlockSizeOptimizationConfig opt_config;
            try {
                opt_config = loadBlockSizeOptimizationConfigFromYaml(config_path);
            } catch (const std::exception& e) {
                std::cerr << "Failed to load config: " << e.what() << "\n";
                return 1;
            }

            auto errors = opt_config.validate(true);
            if (!errors.empty()) {
                for (const auto& err : errors) {
                    std::cerr << "Config error: " << err << "\n";
                }
                return 1;
            }

            CompressionSettings base_settings = settingsFromConfig(opt_config.base);
            PointCloudCompressor compressor(base_settings);

            auto optimal_settings = compressor.findOptimalSettings(opt_config.base.input_file);

            std::cout << "Optimal compression settings:\n";
            std::cout << "  Voxel size       : " << optimal_settings.voxel_size << "\n";
            std::cout << "  Block size       : " << optimal_settings.block_size << "\n";
            auto block_result = compressor.findOptimalBlockSize(opt_config.base.input_file,
                                                                 opt_config.min_block_size,
                                                                 opt_config.max_block_size,
                                                                 opt_config.step_size,
                                                                 opt_config.verbose);
            std::cout << formatBlockSizeSummary(block_result, opt_config.verbose) << "\n";

            std::cout << "Processing completed!!\n";

        } else {
            std::cerr << "Error: Unknown command '" << command << "'\n";
            printUsage(argv[0]);
            return 1;
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
