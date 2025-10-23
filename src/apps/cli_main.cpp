// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_compressor/config/CompressorConfig.hpp"
#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
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

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " <command> [options]\n";
    std::cout << "\nCommands:\n";
    std::cout << "  compress [<input.pcd> <output.h5>]       - Compress a PCD/PLY file into an HDF5 archive\n";
    std::cout << "  optimize <input.pcd>                     - Find optimal compression settings\n";
    std::cout << "\nOptions for compress:\n";
    std::cout << "  --voxel-size <size>     Voxel size (default: 0.01)\n";
    std::cout << "  --block-size <size>     Block size (default: 8)\n";
    std::cout << "  --use-8bit              Force 8-bit indices where possible\n";
    std::cout << "  --raw-hdf5 <path>       Also export raw voxel grid to this HDF5 file\n";
    std::cout << "  --config <path>         Load parameters from YAML configuration\n";
    std::cout << "\nExamples:\n";
    std::cout << "  " << program_name << " compress input.pcd output.h5 --voxel-size 0.005\n";
    std::cout << "  " << program_name << " optimize input.pcd\n";
}

std::string ensureHdf5Extension(std::string path) {
    if (path.size() >= 3) {
        auto dot = path.find_last_of('.');
        if (dot != std::string::npos) {
            auto ext = path.substr(dot);
            if (ext == ".h5" || ext == ".hdf5") {
                return path;
            }
        }
    }
    return path + ".h5";
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
            pointcloud_compressor::config::CompressorConfig loaded_config;
            bool has_loaded_config = false;
            CompressionSettings settings;
            std::string raw_hdf5_path;
            std::optional<std::string> config_path;
            std::vector<std::string> positional_args;

            for (int i = 2; i < argc; ++i) {
                std::string arg = argv[i];
                if (arg == "--config" && i + 1 < argc) {
                    config_path = argv[++i];
                } else if (arg == "--voxel-size" && i + 1 < argc) {
                    settings.voxel_size = std::stof(argv[++i]);
                } else if (arg == "--block-size" && i + 1 < argc) {
                    settings.block_size = std::stoi(argv[++i]);
                } else if (arg == "--use-8bit") {
                    settings.use_8bit_indices = true;
                } else if (arg == "--raw-hdf5" && i + 1 < argc) {
                    raw_hdf5_path = argv[++i];
                } else if (arg.rfind("--", 0) == 0) {
                    std::cerr << "Unknown option: " << arg << "\n";
                    return 1;
                } else {
                    positional_args.push_back(arg);
                }
            }

            if (config_path) {
                loaded_config = pointcloud_compressor::config::loadCompressorConfigFromYaml(*config_path);
                has_loaded_config = true;
                settings = pointcloud_compressor::config::settingsFromConfig(loaded_config);
            }

            std::string input_file;
            std::string output_h5;
            bool save_hdf5 = has_loaded_config ? loaded_config.save_hdf5 : true;
            bool save_raw_hdf5 = has_loaded_config ? loaded_config.save_raw_hdf5 : false;
            if (has_loaded_config) {
                input_file = loaded_config.input_file;
                if (loaded_config.save_hdf5 && !loaded_config.hdf5_output_file.empty()) {
                    output_h5 = ensureHdf5Extension(loaded_config.hdf5_output_file);
                }
                if (loaded_config.save_raw_hdf5 && !loaded_config.raw_hdf5_output_file.empty()) {
                    raw_hdf5_path = loaded_config.raw_hdf5_output_file;
                }
            }

            if (!positional_args.empty()) {
                input_file = positional_args[0];
                if (positional_args.size() > 1) {
                    output_h5 = ensureHdf5Extension(positional_args[1]);
                }
            }

            if (input_file.empty()) {
                std::cerr << "Error: input file is unspecified. Provide positional argument or config file.\n";
                printUsage(argv[0]);
                return 1;
            }

            if (!raw_hdf5_path.empty()) {
                save_raw_hdf5 = true;
            }

            if (save_hdf5) {
                if (output_h5.empty()) {
                    if (!has_loaded_config) {
                        std::cerr << "Error: output archive path is required.\n";
                        printUsage(argv[0]);
                        return 1;
                    }
                    std::cerr << "Error: configuration enabled HDF5 saving but no output path provided.\n";
                    return 1;
                }
            } else {
                output_h5.clear();
            }

            PCCRuntimeHandle* handle = pcc_runtime_create();
            if (!handle) {
                std::cerr << "Failed to initialize compressor runtime.\n";
                return 1;
            }

            PCCCompressionRequest request{};
            request.input_file = input_file.c_str();
            request.voxel_size = settings.voxel_size;
            request.block_size = settings.block_size;
            request.use_8bit_indices = settings.use_8bit_indices;
            request.min_points_threshold = settings.min_points_threshold;
            request.save_hdf5 = save_hdf5;
            request.hdf5_output_path = (save_hdf5 && !output_h5.empty()) ? output_h5.c_str() : nullptr;
            request.save_raw_hdf5 = save_raw_hdf5;
            request.raw_hdf5_output_path = raw_hdf5_path.empty() ? nullptr : raw_hdf5_path.c_str();
            request.bounding_box_margin_ratio = settings.bounding_box_margin_ratio;

            if (request.save_raw_hdf5 && !request.raw_hdf5_output_path) {
                std::cerr << "Error: raw HDF5 output enabled but no path specified.\n";
                pcc_runtime_destroy(handle);
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

            if (save_hdf5) {
                printCompressionSummary(report, output_h5);
            } else {
                std::cout << "Compression successful. No archive was saved (save_hdf5=false).\n";
            }

            pcc_runtime_release_report(handle, &report);
            pcc_runtime_destroy(handle);

            if (!raw_hdf5_path.empty()) {
                std::cout << "  Raw voxel grid   : " << raw_hdf5_path << "\n";
            }

        } else if (command == "optimize") {
            if (argc < 3) {
                std::cerr << "Error: optimize command requires input file\n";
                printUsage(argv[0]);
                return 1;
            }

            const std::string input_file = argv[2];

            PointCloudCompressor compressor;
            auto optimal_settings = compressor.findOptimalSettings(input_file);

            std::cout << "Optimal compression settings:\n";
            std::cout << "  Voxel size       : " << optimal_settings.voxel_size << "\n";
            std::cout << "  Block size       : " << optimal_settings.block_size << "\n";
            std::cout << "  Use 8-bit indices: "
                      << (optimal_settings.use_8bit_indices ? "yes" : "no") << "\n";

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
