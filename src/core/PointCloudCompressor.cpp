#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/model/VoxelGrid.hpp"
#include <filesystem>
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace pointcloud_compressor {

PointCloudCompressor::PointCloudCompressor(const CompressionSettings& settings)
    : settings_(settings) {
    voxel_processor_ = std::make_unique<VoxelProcessor>(settings_.voxel_size,
                                                       settings_.block_size,
                                                       settings_.min_points_threshold,
                                                       settings_.bounding_box_margin_ratio);
    dictionary_builder_ = std::make_unique<PatternDictionaryBuilder>();
    pattern_encoder_ = std::make_unique<PatternEncoder>();
}

PointCloudCompressor::~PointCloudCompressor() {}

CompressionResult PointCloudCompressor::compress(const std::string& input_file, 
                                                const std::string& output_prefix) {
    CompressionResult result;
    result.input_file = input_file;
    result.voxel_size = settings_.voxel_size;
    result.block_size = settings_.block_size;
    
    // Validate input
    if (!validateInputFile(input_file)) {
        result.error_message = "Invalid input file: " + input_file;
        return result;
    }
    
    try {
        auto total_start = std::chrono::high_resolution_clock::now();
        voxel_processor_->setBoundingBoxMarginRatio(settings_.bounding_box_margin_ratio);
        
        // Step 1: Load point cloud
        auto load_start = std::chrono::high_resolution_clock::now();
        PointCloud cloud;
        if (!loadPointCloud(input_file, cloud)) {
            result.error_message = "Failed to load point cloud";
            return result;
        }
        auto load_end = std::chrono::high_resolution_clock::now();
        auto load_time = std::chrono::duration_cast<std::chrono::microseconds>(load_end - load_start).count() / 1000.0;
        
        result.point_count = cloud.points.size();
        result.original_size = cloud.points.size() * sizeof(Point3D);
        result.timings.load_ms = load_time;

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
        
        // Step 2: Voxelize and divide into blocks
        auto voxelize_start = std::chrono::high_resolution_clock::now();
        std::vector<VoxelBlock> blocks;
        VoxelGrid grid;
        if (!voxelizeAndDivideWithGrid(cloud, blocks, grid)) {
            result.error_message = "Failed to voxelize point cloud";
            return result;
        }
        auto voxelize_end = std::chrono::high_resolution_clock::now();
        auto voxelize_time = std::chrono::duration_cast<std::chrono::microseconds>(voxelize_end - voxelize_start).count() / 1000.0;
        
        result.num_blocks = blocks.size();
        result.timings.voxelize_ms = voxelize_time;
        result.voxel_grid = grid;
        
        // Cache the voxel grid for reuse
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
        
        // Step 3: Build dictionary and encode
        auto dict_start = std::chrono::high_resolution_clock::now();
        std::vector<uint64_t> indices;
        if (!buildDictionaryAndEncode(blocks, indices)) {
            result.error_message = "Failed to build dictionary";
            return result;
        }
        auto dict_end = std::chrono::high_resolution_clock::now();
        auto dict_time = std::chrono::duration_cast<std::chrono::microseconds>(dict_end - dict_start).count() / 1000.0;
        result.timings.dictionary_ms = dict_time;
        
        result.num_unique_patterns = dictionary_builder_->getUniquePatternCount();
        result.max_index = dictionary_builder_->getMaxIndex();
        result.index_bit_size = dictionary_builder_->getRequiredIndexBitSize();
        
        // Step 4: Save compressed data with actual grid metadata
        auto save_start = std::chrono::high_resolution_clock::now();
        if (!saveCompressionData(output_prefix, indices, grid, result.index_bit_size)) {
            result.error_message = "Failed to save compressed data";
            return result;
        }
        auto save_end = std::chrono::high_resolution_clock::now();
        auto save_time = std::chrono::duration_cast<std::chrono::microseconds>(save_end - save_start).count() / 1000.0;
        result.timings.save_ms = save_time;
        
        // Store actual compression data for ROS message
        result.block_indices = indices;
        result.pattern_dictionary = dictionary_builder_->getUniquePatterns();
        
        // Get grid information from VoxelGrid
        VoxelCoord dims = grid.getDimensions();
        int x_blocks = (dims.x + settings_.block_size - 1) / settings_.block_size;
        int y_blocks = (dims.y + settings_.block_size - 1) / settings_.block_size;
        int z_blocks = (dims.z + settings_.block_size - 1) / settings_.block_size;
        
        result.blocks_count.x = x_blocks;
        result.blocks_count.y = y_blocks;
        result.blocks_count.z = z_blocks;
        
        // Get grid origin and dimensions from VoxelGrid
        float origin_x, origin_y, origin_z;
        grid.getOrigin(origin_x, origin_y, origin_z);
        result.grid_origin.x = origin_x;
        result.grid_origin.y = origin_y;
        result.grid_origin.z = origin_z;
        
        // Calculate grid dimensions from voxel dimensions
        result.grid_dimensions.x = dims.x * settings_.voxel_size;
        result.grid_dimensions.y = dims.y * settings_.voxel_size;
        result.grid_dimensions.z = dims.z * settings_.voxel_size;
        
        // Calculate compression ratio
        result.compressed_size = indices.size() * (result.index_bit_size / 8);
        result.compressed_size += dictionary_builder_->getUniquePatternCount() * ((settings_.block_size * settings_.block_size * settings_.block_size + 7) / 8);
        
        if (result.original_size > 0) {
            result.compression_ratio = static_cast<float>(result.compressed_size) /
                                       static_cast<float>(result.original_size);
        } else {
            result.compression_ratio = 1.0f;
        }
        result.success = true;
        
        auto total_end = std::chrono::high_resolution_clock::now();
        auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(total_end - total_start).count() / 1000.0;
        result.timings.total_ms = total_time;
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
    // Approximate metadata footprint (grid dims, origin, voxel configuration)
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
        
        // Step 1: Load compression data
        auto load_start = std::chrono::high_resolution_clock::now();
        std::vector<uint64_t> indices;
        VoxelGrid grid;
        if (!loadCompressionData(compressed_prefix, indices, grid)) {
            std::cerr << "Failed to load compression data" << std::endl;
            return false;
        }
        auto load_end = std::chrono::high_resolution_clock::now();
        auto load_time = std::chrono::duration_cast<std::chrono::microseconds>(load_end - load_start).count() / 1000.0;
        std::cout << "[PointCloudCompressor] Load compressed data: " << load_time << " ms" << std::endl;
        
        // Step 2: Reconstruct point cloud
        auto reconstruct_start = std::chrono::high_resolution_clock::now();
        PointCloud cloud;
        if (!reconstructPointCloud(indices, grid, cloud)) {
            std::cerr << "Failed to reconstruct point cloud" << std::endl;
            return false;
        }
        auto reconstruct_end = std::chrono::high_resolution_clock::now();
        auto reconstruct_time = std::chrono::duration_cast<std::chrono::microseconds>(reconstruct_end - reconstruct_start).count() / 1000.0;
        std::cout << "[PointCloudCompressor] Reconstruct point cloud: " << reconstruct_time << " ms (" 
                  << cloud.points.size() << " points)" << std::endl;
        
        // Step 3: Save reconstructed point cloud
        auto save_start = std::chrono::high_resolution_clock::now();
        bool result = PointCloudIO::savePointCloud(output_file, cloud);
        auto save_end = std::chrono::high_resolution_clock::now();
        auto save_time = std::chrono::duration_cast<std::chrono::microseconds>(save_end - save_start).count() / 1000.0;
        std::cout << "[PointCloudCompressor] Save point cloud: " << save_time << " ms" << std::endl;
        
        auto total_end = std::chrono::high_resolution_clock::now();
        auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(total_end - total_start).count() / 1000.0;
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
        
        // Step 1: Load compression data
        auto load_start = std::chrono::high_resolution_clock::now();
        std::vector<uint64_t> indices;
        VoxelGrid metadata_grid;
        if (!loadCompressionData(compressed_prefix, indices, metadata_grid)) {
            std::cerr << "Failed to load compression data" << std::endl;
            return false;
        }
        auto load_end = std::chrono::high_resolution_clock::now();
        auto load_time = std::chrono::duration_cast<std::chrono::microseconds>(load_end - load_start).count() / 1000.0;
        std::cout << "[PointCloudCompressor] Load compressed data: " << load_time << " ms" << std::endl;
        
        // Step 2: Reconstruct voxel grid directly
        auto reconstruct_start = std::chrono::high_resolution_clock::now();
        if (!reconstructVoxelGrid(indices, metadata_grid, grid)) {
            std::cerr << "Failed to reconstruct voxel grid" << std::endl;
            return false;
        }
        auto reconstruct_end = std::chrono::high_resolution_clock::now();
        auto reconstruct_time = std::chrono::duration_cast<std::chrono::microseconds>(reconstruct_end - reconstruct_start).count() / 1000.0;
        
        VoxelCoord dims = grid.getDimensions();
        std::cout << "[PointCloudCompressor] Reconstruct voxel grid: " << reconstruct_time 
                  << " ms (" << dims.x << "x" << dims.y << "x" << dims.z 
                  << ", " << grid.getOccupiedVoxelCount() << " occupied)" << std::endl;
        
        auto total_end = std::chrono::high_resolution_clock::now();
        auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(total_end - total_start).count() / 1000.0;
        std::cout << "[PointCloudCompressor] Total grid decompression time: " << total_time << " ms" << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception during grid decompression: " << e.what() << std::endl;
        return false;
    }
}

CompressionSettings PointCloudCompressor::findOptimalSettings(const std::string& input_file,
                                                            float min_voxel_size,
                                                            float max_voxel_size) {
    PointCloud cloud;
    if (!loadPointCloud(input_file, cloud)) {
        return settings_;  // Return current settings if loading fails
    }
    
    // Find optimal voxel size
    float optimal_voxel_size = voxel_processor_->findOptimalVoxelSize(cloud, 
                                                                     min_voxel_size, 
                                                                     max_voxel_size);
    
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
    
    // Input validation
    if (min_block_size < 1) {
        min_block_size = 1;
    }
    if (max_block_size > 64) {
        max_block_size = 64;  // Reasonable upper limit
    }
    if (min_block_size > max_block_size) {
        std::cerr << "[PointCloudCompressor] Invalid range: min_block_size > max_block_size" << std::endl;
        result.optimal_block_size = -1;
        return result;
    }
    if (step_size <= 0) {
        step_size = 1;
    }
    
    // Load point cloud once
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
        std::cout << "[PointCloudCompressor] Testing block sizes from " << min_block_size 
                  << " to " << max_block_size << " with step " << step_size << std::endl;
    }
    
    // Save current settings
    CompressionSettings original_settings = settings_;
    
    // Test each block size
    for (int block_size = min_block_size; block_size <= max_block_size; block_size += step_size) {
        if (verbose) {
            std::cout << "[PointCloudCompressor] Testing block_size = " << block_size << "..." << std::endl;
        }
        
        // Update block size temporarily
        settings_.block_size = block_size;
        voxel_processor_->setBlockSize(block_size);
        
        try {
            // Voxelize and divide into blocks
            std::vector<VoxelBlock> blocks;
            VoxelGrid grid;
            if (!voxelizeAndDivideWithGrid(cloud, blocks, grid)) {
                if (verbose) {
                    std::cout << "[PointCloudCompressor] Failed to voxelize with block_size = " << block_size << std::endl;
                }
                continue;
            }
            
            // Build dictionary and encode
            std::vector<uint64_t> indices;
            if (!buildDictionaryAndEncode(blocks, indices)) {
                if (verbose) {
                    std::cout << "[PointCloudCompressor] Failed to build dictionary with block_size = " << block_size << std::endl;
                }
                continue;
            }
            
            // Calculate compression size accurately
            size_t original_size = cloud.points.size() * sizeof(Point3D);
            int index_bit_size = dictionary_builder_->getRequiredIndexBitSize();
            size_t compressed_size = indices.size() * (index_bit_size / 8);
            
            // Add dictionary size (pattern_size * unique_patterns)
            size_t pattern_bytes = (block_size * block_size * block_size + 7) / 8;
            size_t unique_patterns = dictionary_builder_->getUniquePatternCount();
            compressed_size += pattern_bytes * unique_patterns;
            
            // Calculate compression ratio
            float compression_ratio = static_cast<float>(compressed_size) / static_cast<float>(original_size);
            
            // Calculate comparison with 1 byte/voxel representation
            // Get total voxel count from grid (including non-occupied)
            VoxelCoord dims = grid.getDimensions();
            size_t total_voxels = static_cast<size_t>(dims.x) * dims.y * dims.z;
            size_t one_byte_per_voxel_size = total_voxels * 1;  // 1 byte per voxel
            
            // Our compression size vs 1 byte/voxel
            float compression_vs_one_byte = static_cast<float>(compressed_size) / static_cast<float>(one_byte_per_voxel_size);
            
            if (verbose) {
                // Calculate detailed statistics
                size_t occupied_voxels = grid.getOccupiedVoxelCount();
                
                // Calculate real-world dimensions
                float origin_x, origin_y, origin_z;
                grid.getOrigin(origin_x, origin_y, origin_z);
                float world_size_x = dims.x * settings_.voxel_size;
                float world_size_y = dims.y * settings_.voxel_size;
                float world_size_z = dims.z * settings_.voxel_size;
                
                // Calculate different storage sizes
                size_t one_bit_per_voxel_size = (total_voxels + 7) / 8;  // Round up for bits to bytes
                
                // Calculate component sizes
                size_t codebook_size = pattern_bytes * unique_patterns;
                size_t index_map_size = indices.size() * (settings_.use_8bit_indices ? 1 : 2);
                
                // Calculate compression ratios
                float compression_vs_one_bit = static_cast<float>(compressed_size) / static_cast<float>(one_bit_per_voxel_size);
                
                std::cout << "\n[PointCloudCompressor] ========== Block size " << block_size 
                          << " Statistics ==========" << std::endl;
                std::cout << "Real-world dimensions: " 
                          << world_size_x << "m x " << world_size_y << "m x " << world_size_z << "m" << std::endl;
                std::cout << "Grid dimensions: " << dims.x << " x " << dims.y << " x " << dims.z << " voxels" << std::endl;
                std::cout << "Total voxels: " << total_voxels << std::endl;
                std::cout << "Occupied voxels: " << occupied_voxels 
                          << " (" << (100.0f * occupied_voxels / total_voxels) << "%)" << std::endl;
                std::cout << "-------- Storage Sizes --------" << std::endl;
                std::cout << "1 byte/voxel map size: " << one_byte_per_voxel_size << " bytes" << std::endl;
                std::cout << "1 bit/voxel map size: " << one_bit_per_voxel_size << " bytes" << std::endl;
                std::cout << "-------- Compressed Map --------" << std::endl;
                std::cout << "Codebook size: " << codebook_size << " bytes (" 
                          << unique_patterns << " patterns x " << pattern_bytes << " bytes)" << std::endl;
                std::cout << "Index map size: " << index_map_size << " bytes (" 
                          << indices.size() << " indices x " << (settings_.use_8bit_indices ? 1 : 2) << " bytes)" << std::endl;
                std::cout << "Total compressed size: " << compressed_size << " bytes" << std::endl;
                std::cout << "-------- Compression Ratios --------" << std::endl;
                std::cout << "vs 1 byte/voxel: " << compression_vs_one_byte 
                          << " (" << (1.0f / compression_vs_one_byte) << "x smaller)" << std::endl;
                std::cout << "vs 1 bit/voxel: " << compression_vs_one_bit 
                          << " (" << (1.0f / compression_vs_one_bit) << "x smaller)" << std::endl;
                std::cout << "vs original points: " << compression_ratio << std::endl;
                std::cout << "=====================================\n" << std::endl;
            }
            
            // Store result
            result.tested_results[block_size] = compression_ratio;
            
            // Update best result if this is better
            if (compression_ratio < result.best_compression_ratio) {
                result.best_compression_ratio = compression_ratio;
                result.optimal_block_size = block_size;
            }
            
        } catch (const std::exception& e) {
            if (verbose) {
                std::cerr << "[PointCloudCompressor] Exception with block_size = " << block_size 
                          << ": " << e.what() << std::endl;
            }
        }
    }
    
    // Restore original settings
    settings_ = original_settings;
    voxel_processor_->setBlockSize(original_settings.block_size);
    
    // Calculate optimization time
    auto end_time = std::chrono::high_resolution_clock::now();
    result.optimization_time_ms = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - start_time).count() / 1000.0;
    
    if (verbose) {
        std::cout << "[PointCloudCompressor] Optimization complete in " << result.optimization_time_ms << " ms" << std::endl;
        if (result.optimal_block_size > 0) {
            std::cout << "[PointCloudCompressor] Optimal block size: " << result.optimal_block_size
                      << " with compression ratio: " << result.best_compression_ratio << std::endl;
        }
    }
    
    // If no valid result was found, set a default
    if (result.optimal_block_size < 0 && !result.tested_results.empty()) {
        // Find the best from what we tested
        for (const auto& [block_size, ratio] : result.tested_results) {
            if (ratio < result.best_compression_ratio) {
                result.best_compression_ratio = ratio;
                result.optimal_block_size = block_size;
            }
        }
    }
    
    return result;
}

void PointCloudCompressor::updateSettings(const CompressionSettings& settings) {
    settings_ = settings;
    voxel_processor_->setVoxelSize(settings_.voxel_size);
    voxel_processor_->setBlockSize(settings_.block_size);
    voxel_processor_->setBoundingBoxMarginRatio(settings_.bounding_box_margin_ratio);
}

CompressionSettings PointCloudCompressor::getSettings() const {
    return settings_;
}

bool PointCloudCompressor::validateInputFile(const std::string& filename) {
    return PointCloudIO::validateFile(filename);
}

size_t PointCloudCompressor::estimateMemoryUsage(const std::string& input_file) {
    PointCloud cloud;
    if (!loadPointCloud(input_file, cloud)) {
        return 0;
    }
    
    // Rough estimate: original data + voxel grid + blocks + dictionary
    size_t point_cloud_size = cloud.points.size() * sizeof(Point3D);
    size_t estimated_voxel_grid = point_cloud_size * 2;  // Conservative estimate
    size_t estimated_blocks = point_cloud_size / 10;     // Rough compression estimate
    
    return point_cloud_size + estimated_voxel_grid + estimated_blocks;
}

std::optional<VoxelGrid> PointCloudCompressor::getCachedVoxelGrid() const {
    return cached_voxel_grid_;
}

void PointCloudCompressor::clearCachedVoxelGrid() {
    cached_voxel_grid_.reset();
}

// Private methods
bool PointCloudCompressor::loadPointCloud(const std::string& filename, PointCloud& cloud) {
    return PointCloudIO::loadPointCloud(filename, cloud);
}

bool PointCloudCompressor::voxelizeAndDivide(const PointCloud& cloud, std::vector<VoxelBlock>& blocks) {
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
    
    return voxel_processor_->divideIntoBlocks(grid, blocks);
}

bool PointCloudCompressor::buildDictionaryAndEncode(const std::vector<VoxelBlock>& blocks,
                                                   std::vector<uint64_t>& indices) {
    // Extract patterns from blocks
    std::vector<std::vector<uint8_t>> patterns;
    patterns.reserve(blocks.size());
    
    for (const auto& block : blocks) {
        patterns.push_back(voxel_processor_->extractPattern(block));
    }
    
    // Build dictionary
    if (!dictionary_builder_->buildDictionary(patterns)) {
        return false;
    }
    
    indices = dictionary_builder_->getPatternIndices();
    return true;
}

bool PointCloudCompressor::saveCompressionData(const std::string& output_prefix,
                                              const std::vector<uint64_t>& indices,
                                              const VoxelGrid& grid,
                                              int index_bit_size) {
    // Save dictionary
    if (!dictionary_builder_->saveDictionary(getDictionaryFilename(output_prefix))) {
        return false;
    }
    
    // Save indices with specified bit size
    if (!pattern_encoder_->encodePatternsWithBitSize(indices, getIndicesFilename(output_prefix), index_bit_size)) {
        return false;
    }
    
    // Save metadata
    std::ofstream meta_file(getMetadataFilename(output_prefix), std::ios::binary);
    if (!meta_file.is_open()) {
        return false;
    }
    
    // Write metadata
    meta_file.write(reinterpret_cast<const char*>(&settings_.voxel_size), sizeof(settings_.voxel_size));
    meta_file.write(reinterpret_cast<const char*>(&settings_.block_size), sizeof(settings_.block_size));
    
    // Write grid dimensions
    VoxelCoord dims = grid.getDimensions();
    meta_file.write(reinterpret_cast<const char*>(&dims.x), sizeof(dims.x));
    meta_file.write(reinterpret_cast<const char*>(&dims.y), sizeof(dims.y));
    meta_file.write(reinterpret_cast<const char*>(&dims.z), sizeof(dims.z));
    
    // Write grid origin
    float origin_x, origin_y, origin_z;
    grid.getOrigin(origin_x, origin_y, origin_z);
    meta_file.write(reinterpret_cast<const char*>(&origin_x), sizeof(origin_x));
    meta_file.write(reinterpret_cast<const char*>(&origin_y), sizeof(origin_y));
    meta_file.write(reinterpret_cast<const char*>(&origin_z), sizeof(origin_z));
    
    return meta_file.good();
}

bool PointCloudCompressor::loadCompressionData(const std::string& compressed_prefix,
                                              std::vector<uint64_t>& indices,
                                              VoxelGrid& grid) {
    // Load dictionary
    if (!dictionary_builder_->loadDictionary(getDictionaryFilename(compressed_prefix))) {
        return false;
    }
    
    // Load indices
    if (!pattern_encoder_->decodePatterns(getIndicesFilename(compressed_prefix), indices)) {
        return false;
    }
    
    // Load metadata
    std::ifstream meta_file(getMetadataFilename(compressed_prefix), std::ios::binary);
    if (!meta_file.is_open()) {
        return false;
    }
    
    // Read metadata
    float voxel_size;
    int block_size;
    meta_file.read(reinterpret_cast<char*>(&voxel_size), sizeof(voxel_size));
    meta_file.read(reinterpret_cast<char*>(&block_size), sizeof(block_size));
    
    // Read grid dimensions
    int dim_x, dim_y, dim_z;
    meta_file.read(reinterpret_cast<char*>(&dim_x), sizeof(dim_x));
    meta_file.read(reinterpret_cast<char*>(&dim_y), sizeof(dim_y));
    meta_file.read(reinterpret_cast<char*>(&dim_z), sizeof(dim_z));
    
    // Read grid origin
    float origin_x, origin_y, origin_z;
    meta_file.read(reinterpret_cast<char*>(&origin_x), sizeof(origin_x));
    meta_file.read(reinterpret_cast<char*>(&origin_y), sizeof(origin_y));
    meta_file.read(reinterpret_cast<char*>(&origin_z), sizeof(origin_z));
    
    if (!meta_file.good()) {
        return false;
    }
    
    // Initialize grid with loaded metadata
    grid.initialize(dim_x, dim_y, dim_z, voxel_size);
    grid.setOrigin(origin_x, origin_y, origin_z);
    
    // Update compressor settings
    settings_.voxel_size = voxel_size;
    settings_.block_size = block_size;
    voxel_processor_->setVoxelSize(voxel_size);
    voxel_processor_->setBlockSize(block_size);
    
    return true;
}

bool PointCloudCompressor::reconstructVoxelGrid(const std::vector<uint64_t>& indices,
                                               const VoxelGrid& metadata_grid,
                                               VoxelGrid& reconstructed_grid) {
    // Get unique patterns from dictionary
    const auto& unique_patterns = dictionary_builder_->getUniquePatterns();
    
    if (unique_patterns.empty() && !indices.empty()) {
        std::cerr << "Error: Dictionary is empty but indices are not" << std::endl;
        return false;
    }
    
    // Initialize grid with metadata
    VoxelCoord dims = metadata_grid.getDimensions();
    float voxel_size = metadata_grid.getVoxelSize();
    reconstructed_grid.initialize(dims.x, dims.y, dims.z, voxel_size);
    
    float origin_x, origin_y, origin_z;
    metadata_grid.getOrigin(origin_x, origin_y, origin_z);
    reconstructed_grid.setOrigin(origin_x, origin_y, origin_z);
    
    // Calculate number of blocks
    int x_blocks = (dims.x + settings_.block_size - 1) / settings_.block_size;
    int y_blocks = (dims.y + settings_.block_size - 1) / settings_.block_size;
    int z_blocks = (dims.z + settings_.block_size - 1) / settings_.block_size;
    
    // Reconstruct blocks from indices and insert into grid
    size_t block_index = 0;
    for (int bz = 0; bz < z_blocks; ++bz) {
        for (int by = 0; by < y_blocks; ++by) {
            for (int bx = 0; bx < x_blocks; ++bx) {
                if (block_index >= indices.size()) {
                    std::cerr << "Error: Not enough indices for all blocks" << std::endl;
                    return false;
                }
                
                uint64_t pattern_index = indices[block_index++];
                
                if (pattern_index >= unique_patterns.size()) {
                    std::cerr << "Error: Pattern index " << pattern_index 
                              << " out of range (dictionary size: " 
                              << unique_patterns.size() << ")" << std::endl;
                    return false;
                }
                
                // Get pattern and reconstruct block
                const auto& pattern = unique_patterns[pattern_index];
                VoxelBlock block(settings_.block_size);
                block.fromBytePattern(pattern);
                block.position = VoxelCoord(bx, by, bz);
                
                // Insert block into grid
                reconstructed_grid.insertBlock(block, bx, by, bz);
            }
        }
    }
    
    return true;
}

bool PointCloudCompressor::reconstructPointCloud(const std::vector<uint64_t>& indices,
                                                const VoxelGrid& metadata_grid,
                                                PointCloud& cloud) {
    cloud.clear();
    
    // Reconstruct the voxel grid first
    VoxelGrid reconstructed_grid;
    if (!reconstructVoxelGrid(indices, metadata_grid, reconstructed_grid)) {
        return false;
    }
    
    // Extract points from reconstructed voxel grid
    return voxel_processor_->reconstructPointCloud(reconstructed_grid, cloud);
}

void PointCloudCompressor::initializeTempPaths(const std::string& output_prefix) {
    // Create temp directory if it doesn't exist
    std::filesystem::create_directories(settings_.temp_directory);
}

void PointCloudCompressor::cleanupTempFiles() {
    // Remove temporary files if needed
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

} // namespace pointcloud_compressor
