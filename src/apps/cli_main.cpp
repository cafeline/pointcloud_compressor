// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "vq_occupancy_compressor/config/CliConfigParser.hpp"
#include "vq_occupancy_compressor/config/ConfigTransforms.hpp"
#include "vq_occupancy_compressor/config/CompressorConfig.hpp"
#include "vq_occupancy_compressor/cli/OptimizeWorkflow.hpp"
#include "vq_occupancy_compressor/report/ReportUtilities.hpp"
#include "vq_occupancy_compressor/core/VqOccupancyCompressor.hpp"
#include "vq_occupancy_compressor/io/Hdf5Writers.hpp"
#include "vq_occupancy_compressor/utils/ErrorAccumulator.hpp"
#include "vq_occupancy_compressor/services/CompressionExecutor.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {

using vq_occupancy_compressor::config::CompressionSetup;
using vq_occupancy_compressor::config::CompressorConfig;
using vq_occupancy_compressor::config::buildCompressionSetup;
using vq_occupancy_compressor::config::loadCompressorConfigFromYaml;
using vq_occupancy_compressor::config::parseConfigPath;
using vq_occupancy_compressor::config::validateCompressionSetup;
using vq_occupancy_compressor::io::CompressionReportBuilder;
using vq_occupancy_compressor::services::runCompression;

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " <command> [options]\n";
    std::cout << "\nCommands:\n";
    std::cout << "  compress <config.yaml>                    - Compress using YAML configuration\n";
    std::cout << "  optimize <config.yaml>                    - Run block size optimization using YAML\n";
}

void printCompressionSummary(const PCCCompressionReport& report, const std::string& output_h5) {
    std::cout << vq_occupancy_compressor::formatCompressionSummary(report) << "\n";
    std::cout << "  Output archive   : " << output_h5 << "\n";
}

}  

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

            vq_occupancy_compressor::utils::ErrorAccumulator error_acc;
            std::string compression_error;
            const bool compression_success = runCompression(
                setup,
                [&](const PCCCompressionReport& report,
                    vq_occupancy_compressor::io::CompressionReportBuilder& builder) {
                    auto map_data = builder.toCompressedMapData(report,
                                                                setup.settings.voxel_size,
                                                                setup.settings.block_size);

                    if (setup.request.save_hdf5 && setup.request.hdf5_output_path) {
                        std::string err;
                        if (!vq_occupancy_compressor::io::writeCompressedMap(setup.config.hdf5_output_file,
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
                        if (vq_occupancy_compressor::io::writeRawVoxelGrid(setup.config.raw_hdf5_output_file,
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

            try {
                const auto result = vq_occupancy_compressor::cli::runOptimizeWorkflow(config_path);

                std::cout << "Optimal compression settings:\n";
                std::cout << "  Voxel size       : " << result.optimal_settings.voxel_size << "\n";
                std::cout << "  Block size       : " << result.optimal_settings.block_size << "\n";
                std::cout << formatBlockSizeSummary(result.block_result, result.verbose) << "\n";
                if (!result.compression_summary.empty()) {
                    std::cout << result.compression_summary << "\n";
                }
                if (!result.hdf5_output_file.empty()) {
                    std::cout << "  Output archive   : " << result.hdf5_output_file << "\n";
                } else {
                    std::cout << "  Output archive   : <not saved>\n";
                }
                if (!result.raw_hdf5_output_file.empty()) {
                    std::cout << "  Raw voxel grid   : " << result.raw_hdf5_output_file << "\n";
                }

                std::cout << "Processing completed!!\n";
            } catch (const std::exception& e) {
                std::cerr << "Optimization failed: " << e.what() << "\n";
                return 1;
            }

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
