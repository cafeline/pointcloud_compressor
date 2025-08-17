#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include <filesystem>
#include <iostream>
#include <fstream>
#include <cmath>

namespace pointcloud_compressor {

PointCloudCompressor::PointCloudCompressor(const CompressionSettings& settings)
    : settings_(settings) {
    voxel_processor_ = std::make_unique<VoxelProcessor>(settings_.voxel_size, settings_.block_size, settings_.min_points_threshold);
    dictionary_builder_ = std::make_unique<PatternDictionaryBuilder>();
    pattern_encoder_ = std::make_unique<PatternEncoder>();
}

PointCloudCompressor::~PointCloudCompressor() {}

CompressionResult PointCloudCompressor::compress(const std::string& input_file, 
                                                const std::string& output_prefix) {
    CompressionResult result;
    
    // Validate input
    if (!validateInputFile(input_file)) {
        result.error_message = "Invalid input file: " + input_file;
        return result;
    }
    
    try {
        // Step 1: Load point cloud
        PointCloud cloud;
        if (!loadPointCloud(input_file, cloud)) {
            result.error_message = "Failed to load point cloud";
            return result;
        }
        
        result.original_size = cloud.points.size() * sizeof(Point3D);
        
        // Step 2: Voxelize and divide into blocks
        std::vector<VoxelBlock> blocks;
        if (!voxelizeAndDivide(cloud, blocks)) {
            result.error_message = "Failed to voxelize point cloud";
            return result;
        }
        
        result.num_blocks = blocks.size();
        
        // Step 3: Build dictionary and encode
        std::vector<uint16_t> indices;
        if (!buildDictionaryAndEncode(blocks, indices)) {
            result.error_message = "Failed to build dictionary";
            return result;
        }
        
        result.num_unique_patterns = dictionary_builder_->getUniquePatternCount();
        
        // Step 4: Save compressed data  
        VoxelGrid dummy_grid; // TODO: Save actual grid metadata
        if (!saveCompressionData(output_prefix, indices, dummy_grid)) {
            result.error_message = "Failed to save compressed data";
            return result;
        }
        
        // Store actual compression data for ROS message
        result.block_indices = indices;
        
        // Calculate basic grid information from point cloud bounds
        if (!cloud.points.empty()) {
            float min_x = cloud.points[0].x, max_x = cloud.points[0].x;
            float min_y = cloud.points[0].y, max_y = cloud.points[0].y;
            float min_z = cloud.points[0].z, max_z = cloud.points[0].z;
            
            for (const auto& point : cloud.points) {
                min_x = std::min(min_x, point.x);
                max_x = std::max(max_x, point.x);
                min_y = std::min(min_y, point.y);
                max_y = std::max(max_y, point.y);
                min_z = std::min(min_z, point.z);
                max_z = std::max(max_z, point.z);
            }
            
            result.grid_dimensions.x = max_x - min_x;
            result.grid_dimensions.y = max_y - min_y;
            result.grid_dimensions.z = max_z - min_z;
            result.grid_origin.x = min_x;
            result.grid_origin.y = min_y;
            result.grid_origin.z = min_z;
        }
        
        // Calculate blocks count (simplified)
        int blocks_per_dim = static_cast<int>(std::ceil(std::cbrt(blocks.size())));
        result.blocks_count.x = blocks_per_dim;
        result.blocks_count.y = blocks_per_dim;
        result.blocks_count.z = std::max(1, static_cast<int>(blocks.size()) / (blocks_per_dim * blocks_per_dim));
        
        // Calculate compression ratio
        result.compressed_size = indices.size() * (settings_.use_8bit_indices ? 1 : 2);
        result.compressed_size += dictionary_builder_->getUniquePatternCount() * 64; // Estimate
        
        result.compression_ratio = static_cast<float>(result.compressed_size) / 
                                  static_cast<float>(result.original_size);
        result.success = true;
        
    } catch (const std::exception& e) {
        result.error_message = "Exception during compression: " + std::string(e.what());
    }
    
    return result;
}

bool PointCloudCompressor::decompress(const std::string& compressed_prefix, 
                                     const std::string& output_file) {
    try {
        // Step 1: Load compression data
        std::vector<uint16_t> indices;
        VoxelGrid grid;
        if (!loadCompressionData(compressed_prefix, indices, grid)) {
            std::cerr << "Failed to load compression data" << std::endl;
            return false;
        }
        
        // Step 2: Reconstruct point cloud
        PointCloud cloud;
        if (!reconstructPointCloud(indices, grid, cloud)) {
            std::cerr << "Failed to reconstruct point cloud" << std::endl;
            return false;
        }
        
        // Step 3: Save reconstructed point cloud
        return PointCloudIO::savePointCloud(output_file, cloud);
        
    } catch (const std::exception& e) {
        std::cerr << "Exception during decompression: " << e.what() << std::endl;
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

void PointCloudCompressor::updateSettings(const CompressionSettings& settings) {
    settings_ = settings;
    voxel_processor_->setVoxelSize(settings_.voxel_size);
    voxel_processor_->setBlockSize(settings_.block_size);
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

bool PointCloudCompressor::buildDictionaryAndEncode(const std::vector<VoxelBlock>& blocks,
                                                   std::vector<uint16_t>& indices) {
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
                                              const std::vector<uint16_t>& indices,
                                              const VoxelGrid& grid) {
    // Save dictionary
    if (!dictionary_builder_->saveDictionary(getDictionaryFilename(output_prefix))) {
        return false;
    }
    
    // Save indices
    if (settings_.use_8bit_indices && pattern_encoder_->canUse8bitIndices(indices)) {
        return pattern_encoder_->encodePatterns8bit(indices, getIndicesFilename(output_prefix));
    } else {
        return pattern_encoder_->encodePatterns(indices, getIndicesFilename(output_prefix));
    }
}

bool PointCloudCompressor::loadCompressionData(const std::string& compressed_prefix,
                                              std::vector<uint16_t>& indices,
                                              VoxelGrid& grid) {
    // Load dictionary
    if (!dictionary_builder_->loadDictionary(getDictionaryFilename(compressed_prefix))) {
        return false;
    }
    
    // Load indices
    return pattern_encoder_->decodePatterns(getIndicesFilename(compressed_prefix), indices);
}

bool PointCloudCompressor::reconstructPointCloud(const std::vector<uint16_t>& indices,
                                                const VoxelGrid& grid,
                                                PointCloud& cloud) {
    // TODO: Implement reconstruction from indices and dictionary
    // This is a simplified stub - full implementation would:
    // 1. Get unique patterns from dictionary
    // 2. Reconstruct blocks using indices
    // 3. Rebuild voxel grid from blocks
    // 4. Extract points from voxel grid
    
    cloud.clear();
    return false;  // Not implemented yet
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