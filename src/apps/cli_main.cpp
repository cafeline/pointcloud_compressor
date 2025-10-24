// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_compressor/config/CliConfigParser.hpp"
#include "pointcloud_compressor/config/CompressorConfig.hpp"
#include "pointcloud_compressor/core/BlockSizeReportFormatter.hpp"
#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/runtime/CompressionReportBuilder.hpp"
#include "pointcloud_compressor/runtime/Hdf5Writers.hpp"
#include "pointcloud_compressor/runtime/RuntimeAPI.hpp"

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
using pointcloud_compressor::HDF5IO;
using pointcloud_compressor::config::BlockSizeOptimizationConfig;
using pointcloud_compressor::config::CompressorConfig;
using pointcloud_compressor::config::loadBlockSizeOptimizationConfigFromYaml;
using pointcloud_compressor::config::loadCompressorConfigFromYaml;
using pointcloud_compressor::config::parseConfigPath;
using pointcloud_compressor::config::settingsFromConfig;
using pointcloud_compressor::config::toCompressionRequest;
using pointcloud_compressor::runtime::CompressionReportBuilder;

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " <command> [options]\n";
    std::cout << "\nCommands:\n";
    std::cout << "  compress <config.yaml>                    - Compress using YAML configuration\n";
    std::cout << "  optimize <config.yaml>                    - Run block size optimization using YAML\n";
}

void printCompressionSummary(const PCCCompressionReport& report, const std::string& output_h5) {
    std::cout << "Compression successful.\n";
    std::cout << "  Output archive   : " << output_h5 << "\n";
    std::cout << "  Compression ratio: " << report.statistics.compression_ratio << "\n";
    std::cout << "  Block count      : " << report.indices.total_blocks << "\n";
    std::cout << "  Dictionary size  : " << report.dictionary.num_patterns
              << " patterns (" << report.dictionary.pattern_size_bytes << " bytes each)\n";
    std::cout << "  Index bit width  : " << static_cast<int>(report.indices.index_bit_size) << " bits\n";
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

            CompressorConfig config;
            try {
                config = loadCompressorConfigFromYaml(config_path);
            } catch (const std::exception& e) {
                std::cerr << "Failed to load config: " << e.what() << "\n";
                return 1;
            }

            auto errors = config.validate(true);
            if (!errors.empty()) {
                for (const auto& err : errors) {
                    std::cerr << "Config error: " << err << "\n";
                }
                return 1;
            }

            CompressionSettings settings = settingsFromConfig(config);
            auto request = toCompressionRequest(config, settings);

            PCCRuntimeHandle* handle = pcc_runtime_create();
            if (!handle) {
                std::cerr << "Failed to initialize compressor runtime.\n";
                return 1;
            }

            auto report = pcc_runtime_compress(handle, &request);
            if (!report.success) {
                const char* error_msg = report.error_message ? report.error_message : "Unknown error";
                std::cerr << "Compression failed: " << error_msg << "\n";
                pcc_runtime_release_report(handle, &report);
                pcc_runtime_destroy(handle);
                return 1;
            }

            CompressionReportBuilder builder;
            auto map_data = builder.toCompressedMapData(report, config.voxel_size, config.block_size);

            std::string write_errors;
            if (config.save_hdf5 && request.hdf5_output_path) {
                if (!pointcloud_compressor::runtime::writeCompressedMap(config.hdf5_output_file, map_data, write_errors)) {
                    std::cerr << write_errors << "\n";
                } else {
                    printCompressionSummary(report, config.hdf5_output_file);
                }
            } else {
                std::cout << "Compression successful. No archive was saved (save_hdf5=false).\n";
            }

            if (config.save_raw_hdf5 && request.raw_hdf5_output_path) {
                write_errors.clear();
                if (pointcloud_compressor::runtime::writeRawVoxelGrid(config.raw_hdf5_output_file, report, write_errors)) {
                    std::cout << "  Raw voxel grid   : " << config.raw_hdf5_output_file << "\n";
                } else {
                    std::cerr << write_errors << "\n";
                }
            }

            pcc_runtime_release_report(handle, &report);
            pcc_runtime_destroy(handle);

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
            std::cout << "  Use 8-bit indices: "
                      << (optimal_settings.use_8bit_indices ? "yes" : "no") << "\n";

            auto block_result = compressor.findOptimalBlockSize(opt_config.base.input_file,
                                                                 opt_config.min_block_size,
                                                                 opt_config.max_block_size,
                                                                 opt_config.step_size,
                                                                 opt_config.verbose);
            std::cout << formatBlockSizeSummary(block_result, opt_config.verbose) << "\n";

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
