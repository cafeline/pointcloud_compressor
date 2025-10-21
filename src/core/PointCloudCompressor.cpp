#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/model/VoxelGrid.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>

namespace pointcloud_compressor {

PointCloudCompressor::PointCloudCompressor(const CompressionSettings& settings)
    : settings_(settings) {
    voxel_processor_ = std::make_unique<VoxelProcessor>(
        settings_.voxel_size,
        settings_.block_size,
        settings_.min_points_threshold,
        settings_.bounding_box_margin_ratio);
    dictionary_builder_ = std::make_unique<PatternDictionaryBuilder>();
    pattern_encoder_ = std::make_unique<PatternEncoder>();
}

PointCloudCompressor::~PointCloudCompressor() = default;

CompressionResult PointCloudCompressor::compress(const std::string& input_file,
                                                const std::string& output_prefix) {
    CompressionResult result;
    result.input_file = input_file;
    result.voxel_size = settings_.voxel_size;
    result.block_size = settings_.block_size;

    if (!validateInputFile(input_file)) {
        result.error_message = "Invalid input file: " + input_file;
        return result;
    }

    try {
        auto total_start = std::chrono::high_resolution_clock::now();
        voxel_processor_->setBoundingBoxMarginRatio(settings_.bounding_box_margin_ratio);

        auto load_start = std::chrono::high_resolution_clock::now();
        PointCloud cloud;
        if (!loadPointCloud(input_file, cloud)) {
            result.error_message = "Failed to load point cloud";
            return result;
        }
        auto load_end = std::chrono::high_resolution_clock::now();
        result.point_count = cloud.points.size();
        result.original_size = cloud.points.size() * sizeof(Point3D);
        result.timings.load_ms =
            std::chrono::duration<double, std::milli>(load_end - load_start).count();

        Point3D original_min{0.0f, 0.0f, 0.0f};
        Point3D original_max{0.0f, 0.0f, 0.0f};
        if (!cloud.points.empty()) {
            original_min = cloud.points.front();
            original_max = cloud.points.front();
            for (const auto& p : cloud.points) {
                original_min.x = std::min(original_min.x, p.x);
                original_min.y = std::min(original_min.y, p.y);
                original_min.z = std::min(original_min.z, p.z);
                original_max.x = std::max(original_max.x, p.x);
                original_max.y = std::max(original_max.y, p.y);
                original_max.z = std::max(original_max.z, p.z);
            }
        }
        const double ratio = static_cast<double>(settings_.bounding_box_margin_ratio);
        result.margin.x = (static_cast<double>(original_max.x) - static_cast<double>(original_min.x)) * ratio;
        result.margin.y = (static_cast<double>(original_max.y) - static_cast<double>(original_min.y)) * ratio;
        result.margin.z = (static_cast<double>(original_max.z) - static_cast<double>(original_min.z)) * ratio;

        auto voxelize_start = std::chrono::high_resolution_clock::now();
        std::vector<VoxelBlock> blocks;
        VoxelGrid grid;
        if (!voxelizeAndDivideWithGrid(cloud, blocks, grid)) {
            result.error_message = "Failed to voxelize point cloud";
            return result;
        }
        auto voxelize_end = std::chrono::high_resolution_clock::now();
        result.num_blocks = blocks.size();
        result.timings.voxelize_ms =
            std::chrono::duration<double, std::milli>(voxelize_end - voxelize_start).count();
        result.voxel_grid = grid;
        cached_voxel_grid_ = grid;

        const auto& voxel_report = voxel_processor_->getLastReport();
        result.voxel_stats.grid_x = voxel_report.grid_x;
        result.voxel_stats.grid_y = voxel_report.grid_y;
        result.voxel_stats.grid_z = voxel_report.grid_z;
        result.voxel_stats.total_voxels = voxel_report.total_voxels;
        result.voxel_stats.occupied_voxels = voxel_report.occupied_voxels_estimate;
        result.voxel_stats.transferred_voxels = voxel_report.occupied_voxels_committed;
        result.voxel_stats.bbox_time_ms = voxel_report.bbox_time_ms;
        result.voxel_stats.grid_init_time_ms = voxel_report.grid_init_time_ms;
        result.voxel_stats.voxel_count_time_ms = voxel_report.voxel_count_time_ms;
        result.voxel_stats.grid_transfer_time_ms = voxel_report.grid_transfer_time_ms;
        result.voxel_stats.voxelization_time_ms = voxel_report.voxelization_time_ms;
        result.voxel_stats.block_count = voxel_report.total_blocks;
        result.voxel_stats.blocks_per_axis.x = voxel_report.blocks_x;
        result.voxel_stats.blocks_per_axis.y = voxel_report.blocks_y;
        result.voxel_stats.blocks_per_axis.z = voxel_report.blocks_z;
        result.voxel_stats.block_division_time_ms = voxel_report.block_division_time_ms;

        auto dict_start = std::chrono::high_resolution_clock::now();
        std::vector<uint64_t> indices;
        if (!buildDictionaryAndEncode(blocks, indices)) {
            result.error_message = "Failed to build dictionary";
            return result;
        }
        auto dict_end = std::chrono::high_resolution_clock::now();
        result.timings.dictionary_ms =
            std::chrono::duration<double, std::milli>(dict_end - dict_start).count();

        result.num_unique_patterns = dictionary_builder_->getUniquePatternCount();
        result.max_index = dictionary_builder_->getMaxIndex();
        result.index_bit_size = dictionary_builder_->getRequiredIndexBitSize();

        auto save_start = std::chrono::high_resolution_clock::now();
        if (!saveCompressionData(output_prefix, indices, grid, result.index_bit_size)) {
            result.error_message = "Failed to save compressed data";
            return result;
        }
        auto save_end = std::chrono::high_resolution_clock::now();
        result.timings.save_ms =
            std::chrono::duration<double, std::milli>(save_end - save_start).count();

        result.block_indices = indices;
        result.pattern_dictionary = dictionary_builder_->getUniquePatterns();

        VoxelCoord dims = grid.getDimensions();
        const int x_blocks = (dims.x + settings_.block_size - 1) / settings_.block_size;
        const int y_blocks = (dims.y + settings_.block_size - 1) / settings_.block_size;
        const int z_blocks = (dims.z + settings_.block_size - 1) / settings_.block_size;
        result.blocks_count = {x_blocks, y_blocks, z_blocks};

        float origin_x = 0.0f, origin_y = 0.0f, origin_z = 0.0f;
        grid.getOrigin(origin_x, origin_y, origin_z);
        result.grid_origin.x = origin_x;
        result.grid_origin.y = origin_y;
        result.grid_origin.z = origin_z;

        result.grid_dimensions.x = dims.x * settings_.voxel_size;
        result.grid_dimensions.y = dims.y * settings_.voxel_size;
        result.grid_dimensions.z = dims.z * settings_.voxel_size;

        const size_t index_bytes = indices.size() * ((result.index_bit_size + 7) / 8);
        size_t dictionary_bytes = 0;
        for (const auto& pattern : result.pattern_dictionary) {
            dictionary_bytes += pattern.size();
        }
        result.compressed_size = index_bytes + dictionary_bytes;

        if (result.original_size > 0) {
            result.compression_ratio =
                static_cast<float>(result.compressed_size) / static_cast<float>(result.original_size);
        } else {
            result.compression_ratio = 1.0f;
        }
        result.success = true;

        auto total_end = std::chrono::high_resolution_clock::now();
        result.timings.total_ms =
            std::chrono::duration<double, std::milli>(total_end - total_start).count();
    } catch (const std::exception& e) {
        result.error_message = "Exception during compression: " + std::string(e.what());
    }

    return result;
}

std::string CompressionResult::formatDetailedReport() const {
    auto formatMs = [](double value) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3) << value;
        return ss.str();
    };
    auto formatRatio = [](double value) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(6) << value;
        return ss.str();
    };
    auto formatBytesWithKb = [](size_t bytes) {
        std::ostringstream ss;
        ss << bytes << " bytes (" << std::fixed << std::setprecision(2)
           << (bytes / 1024.0) << " KB)";
        return ss.str();
    };
    auto formatPercent = [](double value) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2) << value;
        return ss.str();
    };
    auto formatFloat = [](double value, int precision = 3) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(precision) << value;
        return ss.str();
    };

    if (!success) {
        std::ostringstream failed;
        failed << "[REPORT][Compression]\n"
               << "  Status           : failed";
        if (!error_message.empty()) {
            failed << "\n  Error            : " << error_message;
        }
        return failed.str();
    }

    std::ostringstream report;
    report << "\n[REPORT][File IO]\n";
    if (!input_file.empty()) {
        report << "  Input file        : " << input_file << '\n';
    }
    report << "  Points            : " << point_count << '\n'
           << "  Load time         : " << formatMs(timings.load_ms) << " ms\n";

    const uint64_t grid_x = static_cast<uint64_t>(voxel_stats.grid_x);
    const uint64_t grid_y = static_cast<uint64_t>(voxel_stats.grid_y);
    const uint64_t grid_z = static_cast<uint64_t>(voxel_stats.grid_z);
    const uint64_t total_voxels = grid_x * grid_y * grid_z;

    report << "[REPORT][Voxelization]\n"
           << "  Grid dims         : " << voxel_stats.grid_x << " x "
           << voxel_stats.grid_y << " x " << voxel_stats.grid_z;
    if (total_voxels > 0) {
        report << " (" << total_voxels << " voxels)";
    }
    report << '\n'
           << "  Bounding box      : " << formatMs(voxel_stats.bbox_time_ms) << " ms\n"
           << "  Grid init         : " << formatMs(voxel_stats.grid_init_time_ms) << " ms\n"
           << "  Voxel counting    : " << formatMs(voxel_stats.voxel_count_time_ms) << " ms ("
           << voxel_stats.occupied_voxels << " candidate voxels)\n"
           << "  Grid transfer     : " << formatMs(voxel_stats.grid_transfer_time_ms) << " ms ("
           << voxel_stats.transferred_voxels << " voxels set)\n"
           << "  Block division    : " << formatMs(voxel_stats.block_division_time_ms) << " ms -> "
           << voxel_stats.blocks_per_axis.x << " x "
           << voxel_stats.blocks_per_axis.y << " x "
           << voxel_stats.blocks_per_axis.z << " = "
           << voxel_stats.block_count << " blocks\n"
           << "  Voxelize total    : " << formatMs(voxel_stats.voxelization_time_ms) << " ms\n";

    report << "[REPORT][Compression]\n"
           << "  Blocks processed  : " << num_blocks << '\n'
           << "  Codebook patterns : " << num_unique_patterns << '\n'
           << "  Index entries     : " << block_indices.size() << '\n'
           << "  Dictionary build  : " << formatMs(timings.dictionary_ms) << " ms\n"
           << "  Index bit width   : " << index_bit_size << "-bit\n"
           << "  Save data         : " << formatMs(timings.save_ms) << " ms\n"
           << "  Total time        : " << formatMs(timings.total_ms) << " ms\n"
           << "  Compression ratio : " << formatRatio(compression_ratio) << '\n';

    const uint64_t occupied_voxels = static_cast<uint64_t>(
        voxel_stats.transferred_voxels >= 0 ? voxel_stats.transferred_voxels : 0);
    const size_t bytes_per_index = index_bit_size > 0
        ? static_cast<size_t>((index_bit_size + 7) / 8)
        : 0;
    const size_t block_indices_bytes = bytes_per_index * block_indices.size();
    size_t dictionary_bytes = 0;
    for (const auto& pattern : pattern_dictionary) {
        dictionary_bytes += pattern.size();
    }
    const size_t metadata_bytes = 3 * sizeof(int) + 3 * sizeof(float) + sizeof(float) + 2 * sizeof(int);
    const size_t raw_grid_bytes = total_voxels;
    const size_t compressed_total_bytes = block_indices_bytes + dictionary_bytes + metadata_bytes;
    const double memory_reduction = raw_grid_bytes > 0
        ? (1.0 - static_cast<double>(compressed_total_bytes) / static_cast<double>(raw_grid_bytes)) * 100.0
        : 0.0;
    const double occupancy_percent = total_voxels > 0
        ? (static_cast<double>(occupied_voxels) / static_cast<double>(total_voxels)) * 100.0
        : 0.0;

    report << "[REPORT][Memory]\n";
    if (voxel_size > 0.0f) {
        report << "  Voxel size       : " << formatFloat(voxel_size, 3) << " m\n";
    }
    report << "  Raw grid         : " << formatBytesWithKb(raw_grid_bytes) << '\n'
           << "  Occupied voxels  : " << occupied_voxels << " ("
           << formatPercent(occupancy_percent) << "%)\n"
           << "  Block indices    : " << formatBytesWithKb(block_indices_bytes) << '\n'
           << "  Pattern dict     : " << formatBytesWithKb(dictionary_bytes) << '\n'
           << "  Metadata         : " << metadata_bytes << " bytes\n"
           << "  Compressed total : " << formatBytesWithKb(compressed_total_bytes) << '\n'
           << "  Memory reduction : " << formatPercent(memory_reduction) << "%";

    return report.str();
}

bool PointCloudCompressor::decompress(const std::string& compressed_prefix,
                                      const std::string& output_file) {
    try {
        auto total_start = std::chrono::high_resolution_clock::now();

        auto load_start = std::chrono::high_resolution_clock::now();
        std::vector<uint64_t> indices;
        VoxelGrid grid;
        if (!loadCompressionData(compressed_prefix, indices, grid)) {
            std::cerr << "Failed to load compression data" << std::endl;
            return false;
        }
        auto load_end = std::chrono::high_resolution_clock::now();
        auto load_time =
            std::chrono::duration<double, std::milli>(load_end - load_start).count();
        std::cout << "[PointCloudCompressor] Load compressed data: " << load_time << " ms" << std::endl;

        auto reconstruct_start = std::chrono::high_resolution_clock::now();
        PointCloud cloud;
        if (!reconstructPointCloud(indices, grid, cloud)) {
            std::cerr << "Failed to reconstruct point cloud" << std::endl;
            return false;
        }
        auto reconstruct_end = std::chrono::high_resolution_clock::now();
        auto reconstruct_time =
            std::chrono::duration<double, std::milli>(reconstruct_end - reconstruct_start).count();
        std::cout << "[PointCloudCompressor] Reconstruct point cloud: " << reconstruct_time << " ms ("
                  << cloud.points.size() << " points)" << std::endl;

        auto save_start = std::chrono::high_resolution_clock::now();
        bool result = PointCloudIO::savePointCloud(output_file, cloud);
        auto save_end = std::chrono::high_resolution_clock::now();
        auto save_time =
            std::chrono::duration<double, std::milli>(save_end - save_start).count();
        std::cout << "[PointCloudCompressor] Save point cloud: " << save_time << " ms" << std::endl;

        auto total_end = std::chrono::high_resolution_clock::now();
        auto total_time =
            std::chrono::duration<double, std::milli>(total_end - total_start).count();
        std::cout << "[PointCloudCompressor] Total decompression time: " << total_time << " ms" << std::endl;

        return result;

    } catch (const std::exception& e) {
        std::cerr << "Exception during decompression: " << e.what() << std::endl;
        return false;
    }
}

bool PointCloudCompressor::decompressToGrid(const std::string& compressed_prefix,
                                            VoxelGrid& grid) {
    try {
        auto total_start = std::chrono::high_resolution_clock::now();

        auto load_start = std::chrono::high_resolution_clock::now();
        std::vector<uint64_t> indices;
        VoxelGrid metadata_grid;
        if (!loadCompressionData(compressed_prefix, indices, metadata_grid)) {
            std::cerr << "Failed to load compression data" << std::endl;
            return false;
        }
        auto load_end = std::chrono::high_resolution_clock::now();
        auto load_time =
            std::chrono::duration<double, std::milli>(load_end - load_start).count();
        std::cout << "[PointCloudCompressor] Load compressed data: " << load_time << " ms" << std::endl;

        auto reconstruct_start = std::chrono::high_resolution_clock::now();
        if (!reconstructVoxelGrid(indices, metadata_grid, grid)) {
            std::cerr << "Failed to reconstruct voxel grid" << std::endl;
            return false;
        }
        auto reconstruct_end = std::chrono::high_resolution_clock::now();
        auto reconstruct_time =
            std::chrono::duration<double, std::milli>(reconstruct_end - reconstruct_start).count();
        std::cout << "[PointCloudCompressor] Reconstruct voxel grid: "
                  << reconstruct_time << " ms" << std::endl;

        auto total_end = std::chrono::high_resolution_clock::now();
        auto total_time =
            std::chrono::duration<double, std::milli>(total_end - total_start).count();
        std::cout << "[PointCloudCompressor] Total decomposition time: "
                  << total_time << " ms" << std::endl;

        return true;

    } catch (const std::exception& e) {
        std::cerr << "Exception during voxel grid reconstruction: " << e.what() << std::endl;
        return false;
    }
}

CompressionSettings PointCloudCompressor::findOptimalSettings(const std::string& input_file,
                                                             float min_voxel_size,
                                                             float max_voxel_size) {
    PointCloud cloud;
    if (!loadPointCloud(input_file, cloud)) {
        return settings_;
    }

    const float optimal_voxel_size = voxel_processor_->findOptimalVoxelSize(
        cloud, min_voxel_size, max_voxel_size);

    CompressionSettings optimal_settings = settings_;
    optimal_settings.voxel_size = optimal_voxel_size;
    return optimal_settings;
}

BlockSizeOptimizationResult PointCloudCompressor::findOptimalBlockSize(
    const std::string& input_file,
    int min_block_size,
    int max_block_size,
    int step_size,
    bool verbose) {
    BlockSizeOptimizationResult result;
    auto start_time = std::chrono::high_resolution_clock::now();

    min_block_size = std::max(1, min_block_size);
    max_block_size = std::min(64, max_block_size);
    if (min_block_size > max_block_size) {
        std::cerr << "[PointCloudCompressor] Invalid range: min_block_size > max_block_size" << std::endl;
        result.optimal_block_size = -1;
        return result;
    }
    if (step_size <= 0) {
        step_size = 1;
    }

    PointCloud cloud;
    if (!loadPointCloud(input_file, cloud)) {
        std::cerr << "[PointCloudCompressor] Failed to load point cloud from " << input_file << std::endl;
        result.optimal_block_size = -1;
        return result;
    }

    if (cloud.points.empty()) {
        std::cerr << "[PointCloudCompressor] Point cloud is empty" << std::endl;
        result.optimal_block_size = -1;
        return result;
    }

    if (verbose) {
        std::cout << "[PointCloudCompressor] Starting block size optimization..." << std::endl;
        std::cout << "[PointCloudCompressor] Testing block sizes from "
                  << min_block_size << " to " << max_block_size
                  << " with step " << step_size << std::endl;
    }

    const CompressionSettings original_settings = settings_;
    float best_ratio = std::numeric_limits<float>::max();
    int best_block_size = -1;

    for (int block_size = min_block_size; block_size <= max_block_size; block_size += step_size) {
        if (verbose) {
            std::cout << "[PointCloudCompressor] Testing block_size = "
                      << block_size << "..." << std::endl;
        }

        settings_.block_size = block_size;
        voxel_processor_->setBlockSize(block_size);

        try {
            std::vector<VoxelBlock> blocks;
            VoxelGrid grid;
            if (!voxelizeAndDivideWithGrid(cloud, blocks, grid)) {
                if (verbose) {
                    std::cout << "[PointCloudCompressor] Failed to voxelize with block_size = "
                              << block_size << std::endl;
                }
                continue;
            }

            std::vector<uint64_t> indices;
            if (!buildDictionaryAndEncode(blocks, indices)) {
                if (verbose) {
                    std::cout << "[PointCloudCompressor] Failed to build dictionary with block_size = "
                              << block_size << std::endl;
                }
                continue;
            }

            const size_t original_size = cloud.points.size() * sizeof(Point3D);
            const int index_bit_size = dictionary_builder_->getRequiredIndexBitSize();
            size_t compressed_size = indices.size() * (index_bit_size / 8);
            const size_t pattern_bytes = (block_size * block_size * block_size + 7) / 8;
            const size_t unique_patterns = dictionary_builder_->getUniquePatternCount();
            compressed_size += pattern_bytes * unique_patterns;

            const float compression_ratio =
                static_cast<float>(compressed_size) / static_cast<float>(original_size);

            result.tested_results[block_size] = compression_ratio;

            if (compression_ratio < best_ratio) {
                best_ratio = compression_ratio;
                best_block_size = block_size;
            }

            if (verbose) {
                std::cout << "[PointCloudCompressor] block_size=" << block_size
                          << " -> ratio=" << compression_ratio << std::endl;
            }

        } catch (const std::exception& e) {
            if (verbose) {
                std::cout << "[PointCloudCompressor] Exception for block_size = "
                          << block_size << ": " << e.what() << std::endl;
            }
        }
    }

    settings_ = original_settings;
    voxel_processor_->setBlockSize(settings_.block_size);

    result.optimal_block_size = best_block_size;
    result.best_compression_ratio = best_ratio;
    result.optimization_time_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - start_time).count();
    return result;
}

void PointCloudCompressor::updateSettings(const CompressionSettings& settings) {
    settings_ = settings;
    voxel_processor_->setVoxelSize(settings_.voxel_size);
    voxel_processor_->setBlockSize(settings_.block_size);
    voxel_processor_->setMinPointsThreshold(settings_.min_points_threshold);
    voxel_processor_->setBoundingBoxMarginRatio(settings_.bounding_box_margin_ratio);
}

CompressionSettings PointCloudCompressor::getSettings() const {
    return settings_;
}

bool PointCloudCompressor::validateInputFile(const std::string& filename) {
    if (filename.empty()) {
        return false;
    }
    if (!std::filesystem::exists(filename)) {
        return false;
    }
    return PointCloudIO::validateFile(filename);
}

size_t PointCloudCompressor::estimateMemoryUsage(const std::string& input_file) {
    PointCloud cloud;
    if (!PointCloudIO::loadPointCloud(input_file, cloud)) {
        return 0;
    }

    const size_t num_points = cloud.points.size();
    const size_t voxel_memory = static_cast<size_t>(
        std::ceil(static_cast<double>(num_points) / std::max(1, settings_.min_points_threshold)));
    const size_t pattern_memory = settings_.block_size * settings_.block_size * settings_.block_size;
    return (num_points * sizeof(Point3D)) + voxel_memory + pattern_memory;
}

std::optional<VoxelGrid> PointCloudCompressor::getCachedVoxelGrid() const {
    return cached_voxel_grid_;
}

void PointCloudCompressor::clearCachedVoxelGrid() {
    cached_voxel_grid_.reset();
}

bool PointCloudCompressor::loadPointCloud(const std::string& filename, PointCloud& cloud) {
    return PointCloudIO::loadPointCloud(filename, cloud);
}

bool PointCloudCompressor::voxelizeAndDivide(const PointCloud& cloud,
                                             std::vector<VoxelBlock>& blocks) {
    VoxelGrid grid;
    if (!voxel_processor_->voxelizePointCloud(cloud, grid)) {
        return false;
    }
    return voxel_processor_->divideIntoBlocks(grid, blocks);
}

bool PointCloudCompressor::voxelizeAndDivideWithGrid(const PointCloud& cloud,
                                                     std::vector<VoxelBlock>& blocks,
                                                     VoxelGrid& grid) {
    if (!voxel_processor_->voxelizePointCloud(cloud, grid)) {
        return false;
    }
    if (!voxel_processor_->divideIntoBlocks(grid, blocks)) {
        return false;
    }
    return true;
}

bool PointCloudCompressor::buildDictionaryAndEncode(const std::vector<VoxelBlock>& blocks,
                                                    std::vector<uint64_t>& indices) {
    std::vector<std::vector<uint8_t>> patterns;
    patterns.reserve(blocks.size());
    for (const auto& block : blocks) {
        patterns.push_back(voxel_processor_->extractPattern(block));
    }

    if (!dictionary_builder_->buildDictionary(patterns)) {
        return false;
    }

    const auto& pattern_indices = dictionary_builder_->getPatternIndices();
    indices.assign(pattern_indices.begin(), pattern_indices.end());

    return true;
}

bool PointCloudCompressor::saveCompressionData(const std::string& output_prefix,
                                               const std::vector<uint64_t>& indices,
                                               const VoxelGrid& grid,
                                               int index_bit_size) {
    if (!dictionary_builder_->saveDictionary(getDictionaryFilename(output_prefix))) {
        return false;
    }

    if (!pattern_encoder_->encodePatternsWithBitSize(indices,
                                                     getIndicesFilename(output_prefix),
                                                     index_bit_size)) {
        return false;
    }

    std::ofstream meta_file(getMetadataFilename(output_prefix), std::ios::binary);
    if (!meta_file.is_open()) {
        return false;
    }

    meta_file.write(reinterpret_cast<const char*>(&settings_.voxel_size), sizeof(settings_.voxel_size));
    meta_file.write(reinterpret_cast<const char*>(&settings_.block_size), sizeof(settings_.block_size));

    VoxelCoord dims = grid.getDimensions();
    meta_file.write(reinterpret_cast<const char*>(&dims.x), sizeof(dims.x));
    meta_file.write(reinterpret_cast<const char*>(&dims.y), sizeof(dims.y));
    meta_file.write(reinterpret_cast<const char*>(&dims.z), sizeof(dims.z));

    float origin_x = 0.0f, origin_y = 0.0f, origin_z = 0.0f;
    grid.getOrigin(origin_x, origin_y, origin_z);
    meta_file.write(reinterpret_cast<const char*>(&origin_x), sizeof(origin_x));
    meta_file.write(reinterpret_cast<const char*>(&origin_y), sizeof(origin_y));
    meta_file.write(reinterpret_cast<const char*>(&origin_z), sizeof(origin_z));

    return meta_file.good();
}

bool PointCloudCompressor::loadCompressionData(const std::string& compressed_prefix,
                                               std::vector<uint64_t>& indices,
                                               VoxelGrid& grid) {
    if (!dictionary_builder_->loadDictionary(getDictionaryFilename(compressed_prefix))) {
        return false;
    }

    if (!pattern_encoder_->decodePatterns(getIndicesFilename(compressed_prefix), indices)) {
        return false;
    }

    std::ifstream meta_file(getMetadataFilename(compressed_prefix), std::ios::binary);
    if (!meta_file.is_open()) {
        return false;
    }

    float voxel_size = settings_.voxel_size;
    int block_size = settings_.block_size;
    meta_file.read(reinterpret_cast<char*>(&voxel_size), sizeof(voxel_size));
    meta_file.read(reinterpret_cast<char*>(&block_size), sizeof(block_size));

    int dim_x = 0, dim_y = 0, dim_z = 0;
    meta_file.read(reinterpret_cast<char*>(&dim_x), sizeof(dim_x));
    meta_file.read(reinterpret_cast<char*>(&dim_y), sizeof(dim_y));
    meta_file.read(reinterpret_cast<char*>(&dim_z), sizeof(dim_z));

    float origin_x = 0.0f, origin_y = 0.0f, origin_z = 0.0f;
    meta_file.read(reinterpret_cast<char*>(&origin_x), sizeof(origin_x));
    meta_file.read(reinterpret_cast<char*>(&origin_y), sizeof(origin_y));
    meta_file.read(reinterpret_cast<char*>(&origin_z), sizeof(origin_z));

    if (!meta_file.good()) {
        return false;
    }

    grid.initialize(dim_x, dim_y, dim_z, voxel_size);
    grid.setOrigin(origin_x, origin_y, origin_z);

    settings_.voxel_size = voxel_size;
    settings_.block_size = block_size;
    voxel_processor_->setVoxelSize(voxel_size);
    voxel_processor_->setBlockSize(block_size);

    return true;
}

bool PointCloudCompressor::reconstructPointCloud(const std::vector<uint64_t>& indices,
                                                 const VoxelGrid& grid,
                                                 PointCloud& cloud) {
    cloud.clear();

    VoxelGrid reconstructed_grid;
    if (!reconstructVoxelGrid(indices, grid, reconstructed_grid)) {
        return false;
    }

    return voxel_processor_->reconstructPointCloud(reconstructed_grid, cloud);
}

bool PointCloudCompressor::reconstructVoxelGrid(const std::vector<uint64_t>& indices,
                                                const VoxelGrid& metadata_grid,
                                                VoxelGrid& reconstructed_grid) {
    const auto& unique_patterns = dictionary_builder_->getUniquePatterns();
    if (unique_patterns.empty() && !indices.empty()) {
        std::cerr << "Error: Dictionary is empty but indices are not" << std::endl;
        return false;
    }

    VoxelCoord dims = metadata_grid.getDimensions();
    const float voxel_size = metadata_grid.getVoxelSize();
    reconstructed_grid.initialize(dims.x, dims.y, dims.z, voxel_size);

    float origin_x = 0.0f, origin_y = 0.0f, origin_z = 0.0f;
    metadata_grid.getOrigin(origin_x, origin_y, origin_z);
    reconstructed_grid.setOrigin(origin_x, origin_y, origin_z);

    const int x_blocks = (dims.x + settings_.block_size - 1) / settings_.block_size;
    const int y_blocks = (dims.y + settings_.block_size - 1) / settings_.block_size;
    const int z_blocks = (dims.z + settings_.block_size - 1) / settings_.block_size;

    size_t block_index = 0;
    for (int bz = 0; bz < z_blocks; ++bz) {
        for (int by = 0; by < y_blocks; ++by) {
            for (int bx = 0; bx < x_blocks; ++bx) {
                if (block_index >= indices.size()) {
                    std::cerr << "Error: Not enough indices for all blocks" << std::endl;
                    return false;
                }

                const uint64_t pattern_index = indices[block_index++];
                if (pattern_index >= unique_patterns.size()) {
                    std::cerr << "Error: Pattern index out of bounds" << std::endl;
                    return false;
                }

                const auto& pattern = unique_patterns[pattern_index];
                VoxelBlock block(settings_.block_size);
                block.position = VoxelCoord(bx, by, bz);
                block.fromBytePattern(pattern);
                reconstructed_grid.insertBlock(block, bx, by, bz);
            }
        }
    }

    return true;
}

void PointCloudCompressor::initializeTempPaths(const std::string& /*output_prefix*/) {
    if (!settings_.temp_directory.empty()) {
        std::filesystem::create_directories(settings_.temp_directory);
    }
}

void PointCloudCompressor::cleanupTempFiles() {
    // No-op: temporary files handled by caller
}

std::string PointCloudCompressor::getBlocksFilename(const std::string& prefix) const {
    return prefix + "_blocks.bin";
}

std::string PointCloudCompressor::getDictionaryFilename(const std::string& prefix) const {
    return prefix + "_dict.bin";
}

std::string PointCloudCompressor::getIndicesFilename(const std::string& prefix) const {
    return prefix + "_indices.bin";
}

std::string PointCloudCompressor::getMetadataFilename(const std::string& prefix) const {
    return prefix + "_meta.bin";
}

}  // namespace pointcloud_compressor
