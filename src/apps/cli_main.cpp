#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include <iostream>
#include <string>

namespace {

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " <command> [options]\n";
    std::cout << "\nCommands:\n";
    std::cout << "  compress <input.pcd> <output_prefix>    - Compress a PCD file\n";
    std::cout << "  decompress <input_prefix> <output.pcd>  - Decompress to PCD file\n";
    std::cout << "  optimize <input.pcd>                    - Find optimal compression settings\n";
    std::cout << "\nOptions for compress:\n";
    std::cout << "  --voxel-size <size>     Voxel size (default: 0.01)\n";
    std::cout << "  --block-size <size>     Block size (default: 8)\n";
    std::cout << "  --use-8bit              Use 8-bit indices if possible\n";
    std::cout << "\nExamples:\n";
    std::cout << "  " << program_name << " compress input.pcd output\n";
    std::cout << "  " << program_name << " compress input.pcd output --voxel-size 0.005\n";
    std::cout << "  " << program_name << " decompress output input_restored.pcd\n";
    std::cout << "  " << program_name << " optimize input.pcd\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string command = argv[1];

    try {
        if (command == "compress") {
            if (argc < 4) {
                std::cerr << "Error: compress command requires input and output arguments\n";
                printUsage(argv[0]);
                return 1;
            }

            std::string input_file = argv[2];
            std::string output_prefix = argv[3];

            pointcloud_compressor::CompressionSettings settings;
            for (int i = 4; i < argc; ++i) {
                std::string arg = argv[i];
                if (arg == "--voxel-size" && i + 1 < argc) {
                    settings.voxel_size = std::stof(argv[++i]);
                } else if (arg == "--block-size" && i + 1 < argc) {
                    settings.block_size = std::stoi(argv[++i]);
                } else if (arg == "--use-8bit") {
                    settings.use_8bit_indices = true;
                }
            }

            pointcloud_compressor::PointCloudCompressor compressor(settings);
            auto result = compressor.compress(input_file, output_prefix);

            if (result.success) {
                std::cout << "Compression successful!\n";
                std::cout << result.formatDetailedReport() << std::endl;
            } else {
                std::cerr << "Compression failed: " << result.error_message << "\n";
                return 1;
            }

        } else if (command == "decompress") {
            if (argc < 4) {
                std::cerr << "Error: decompress command requires input prefix and output file\n";
                printUsage(argv[0]);
                return 1;
            }

            std::string input_prefix = argv[2];
            std::string output_file = argv[3];

            pointcloud_compressor::PointCloudCompressor compressor;
            bool success = compressor.decompress(input_prefix, output_file);

            if (success) {
                std::cout << "Decompression successful! Output: " << output_file << "\n";
            } else {
                std::cerr << "Decompression failed\n";
                return 1;
            }

        } else if (command == "optimize") {
            if (argc < 3) {
                std::cerr << "Error: optimize command requires input file\n";
                printUsage(argv[0]);
                return 1;
            }

            std::string input_file = argv[2];

            pointcloud_compressor::PointCloudCompressor compressor;
            auto optimal_settings = compressor.findOptimalSettings(input_file);

            std::cout << "Optimal compression settings:\n";
            std::cout << "  Voxel size: " << optimal_settings.voxel_size << "\n";
            std::cout << "  Block size: " << optimal_settings.block_size << "\n";
            std::cout << "  Use 8-bit indices: " << (optimal_settings.use_8bit_indices ? "yes" : "no") << "\n";

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
