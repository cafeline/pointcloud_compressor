#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <filesystem>
#include <chrono>
#include <algorithm>

#include "pointcloud_compressor/msg/pattern_dictionary.hpp"
#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/core/VoxelProcessor.hpp"
#include "pointcloud_compressor/io/PointCloudIO.hpp"

class PointCloudCompressorNode : public rclcpp::Node
{
public:
    PointCloudCompressorNode() : Node("pointcloud_compressor_node"), compressed_published_(false)
    {
        // Initialize parameters and setup
        declareParameters();
        getParameters();
        
        if (!validateConfiguration()) {
            return;
        }
        
        setupPublishers();
        setupTimer();
        
        RCLCPP_INFO(this->get_logger(), "PointCloud Compressor Node initialized - Input: %s, Voxel: %.3f", 
                    input_file_.c_str(), settings_.voxel_size);
    }

private:
    void declareParameters()
    {
        this->declare_parameter("input_file", "");
        this->declare_parameter("input_pcd_file", "");  // Backward compatibility
        this->declare_parameter("voxel_size", 0.01);
        this->declare_parameter("block_size", 8);
        this->declare_parameter("use_8bit_indices", false);
        this->declare_parameter("min_points_threshold", 1);
        this->declare_parameter("publish_once", true);
        this->declare_parameter("publish_interval_ms", 1000);
        this->declare_parameter("publish_occupied_voxel_markers", false);
    }
    
    void getParameters()
    {
        // Get input file (with backward compatibility)
        input_file_ = this->get_parameter("input_file").as_string();
        if (input_file_.empty()) {
            input_file_ = this->get_parameter("input_pcd_file").as_string();
        }
        
        // Get compression settings
        settings_.voxel_size = this->get_parameter("voxel_size").as_double();
        settings_.block_size = this->get_parameter("block_size").as_int();
        settings_.use_8bit_indices = this->get_parameter("use_8bit_indices").as_bool();
        settings_.min_points_threshold = this->get_parameter("min_points_threshold").as_int();
        
        // Get publishing settings
        publish_once_ = this->get_parameter("publish_once").as_bool();
        publish_interval_ms_ = this->get_parameter("publish_interval_ms").as_int();
        publish_occupied_voxel_markers_ = this->get_parameter("publish_occupied_voxel_markers").as_bool();
    }
    
    bool validateConfiguration()
    {
        // Validate input file
        if (input_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No input file specified. Use parameter 'input_file' or 'input_pcd_file'");
            return false;
        }
        
        // Initialize compressor
        compressor_ = std::make_unique<pointcloud_compressor::PointCloudCompressor>(settings_);
        
        if (!compressor_->validateInputFile(input_file_)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid input file: %s", input_file_.c_str());
            return false;
        }
        
        return true;
    }
    
    void setupPublishers()
    {
        // Pattern dictionary publisher
        pattern_dict_pub_ = this->create_publisher<pointcloud_compressor::msg::PatternDictionary>(
            "pattern_dictionary", 10);
        
        // Occupied voxel marker publisher (optional)
        if (publish_occupied_voxel_markers_) {
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "occupied_voxel_markers", 10);
        }
    }
    
    void setupTimer()
    {
        if (publish_once_) {
            // Compress and publish once immediately
            compressAndPublish();
        } else {
            // Create timer for periodic publishing
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(publish_interval_ms_),
                std::bind(&PointCloudCompressorNode::compressAndPublish, this));
        }
    }

    void compressAndPublish()
    {
        if (compressed_published_ && publish_once_) {
            return;
        }

        // Perform compression
        auto compression_result = performCompression();
        if (!compression_result.success) {
            RCLCPP_ERROR(this->get_logger(), "Compression failed: %s", compression_result.error_message.c_str());
            return;
        }

        // Publish results
        publishPatternDictionary(compression_result);
        publishOccupiedVoxelMarkers();
        
        compressed_published_ = true;
    }
    
    pointcloud_compressor::CompressionResult performCompression()
    {
        // Load point cloud and calculate statistics
        pointcloud_compressor::PointCloud cloud;
        if (pointcloud_compressor::PointCloudIO::loadPointCloud(input_file_, cloud)) {
            calculateAndLogPointCloudStatistics(cloud);
        }
        
        auto start_time = std::chrono::high_resolution_clock::now();
        std::string temp_prefix = "/tmp/compressed_" + std::to_string(this->now().nanoseconds());
        
        auto compression_result = compressor_->compress(input_file_, temp_prefix);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto compression_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if (compression_result.success) {
            // Determine index encoding efficiency
            uint16_t max_index = 0;
            if (!compression_result.block_indices.empty()) {
                max_index = *std::max_element(compression_result.block_indices.begin(), 
                                             compression_result.block_indices.end());
            }
            bool using_8bit = (max_index < 256);
            
            RCLCPP_INFO(this->get_logger(), "Compression completed: %ld ms, ratio %.3f, patterns %zu, %s encoding", 
                        compression_duration.count(), compression_result.compression_ratio, 
                        compression_result.num_unique_patterns, using_8bit ? "8-bit" : "16-bit");
            
            // Calculate and log occupancy grid map size
            calculateAndLogOccupancyGridSize(compression_result);
        }
        
        // Clean up temporary files
        cleanupTempFiles(temp_prefix);
        
        return compression_result;
    }
    
    void publishPatternDictionary(const pointcloud_compressor::CompressionResult& result)
    {
        auto start = std::chrono::high_resolution_clock::now();
        auto msg = createPatternDictionaryMessage(result);
        auto create_end = std::chrono::high_resolution_clock::now();
        
        pattern_dict_pub_->publish(msg);
        auto publish_end = std::chrono::high_resolution_clock::now();
        
        auto create_time = std::chrono::duration_cast<std::chrono::microseconds>(create_end - start).count() / 1000.0;
        auto publish_time = std::chrono::duration_cast<std::chrono::microseconds>(publish_end - create_end).count() / 1000.0;
        
        RCLCPP_INFO(this->get_logger(), "[ROS Node] Create message: %.2f ms, Publish: %.2f ms", 
                    create_time, publish_time);
    }

    pointcloud_compressor::msg::PatternDictionary createPatternDictionaryMessage(
        const pointcloud_compressor::CompressionResult& result)
    {
        auto msg = pointcloud_compressor::msg::PatternDictionary();

        // Set header
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";

        // Set compression settings
        msg.voxel_size = settings_.voxel_size;
        msg.block_size = static_cast<uint32_t>(settings_.block_size);
        msg.use_8bit_indices = settings_.use_8bit_indices;
        msg.min_points_threshold = static_cast<uint32_t>(settings_.min_points_threshold);

        // Set voxel grid information
        msg.voxel_grid_dimensions.x = result.grid_dimensions.x;
        msg.voxel_grid_dimensions.y = result.grid_dimensions.y;
        msg.voxel_grid_dimensions.z = result.grid_dimensions.z;
        msg.voxel_grid_origin.x = result.grid_origin.x;
        msg.voxel_grid_origin.y = result.grid_origin.y;
        msg.voxel_grid_origin.z = result.grid_origin.z;

        // Set block information
        msg.total_blocks = static_cast<uint32_t>(result.num_blocks);
        msg.blocks_x = static_cast<uint32_t>(result.blocks_count.x);
        msg.blocks_y = static_cast<uint32_t>(result.blocks_count.y);
        msg.blocks_z = static_cast<uint32_t>(result.blocks_count.z);

        // Determine index bit size based on maximum index value
        uint16_t max_index = 0;
        if (!result.block_indices.empty()) {
            max_index = *std::max_element(result.block_indices.begin(), result.block_indices.end());
        }
        
        // Set index encoding type
        if (max_index < 256) {
            msg.index_bit_size = 8;
            // Pack as 8-bit indices
            msg.block_indices_data.clear();
            msg.block_indices_data.reserve(result.block_indices.size());
            for (uint16_t idx : result.block_indices) {
                msg.block_indices_data.push_back(static_cast<uint8_t>(idx));
            }
        } else {
            msg.index_bit_size = 16;
            // Pack as 16-bit indices (little-endian)
            msg.block_indices_data.clear();
            msg.block_indices_data.reserve(result.block_indices.size() * 2);
            for (uint16_t idx : result.block_indices) {
                msg.block_indices_data.push_back(static_cast<uint8_t>(idx & 0xFF));         // Low byte
                msg.block_indices_data.push_back(static_cast<uint8_t>((idx >> 8) & 0xFF)); // High byte
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Using %u-bit index encoding (max index: %u)", 
                    msg.index_bit_size, max_index);

        // Set pattern dictionary
        msg.num_patterns = static_cast<uint32_t>(result.num_unique_patterns);
        msg.pattern_size_bytes = 64;  // 8x8x8 block = 512 bits = 64 bytes

        // Set pattern data
        if (!result.pattern_dictionary.empty()) {
            msg.dictionary_data.clear();
            for (const auto& pattern : result.pattern_dictionary) {
                msg.dictionary_data.insert(msg.dictionary_data.end(), pattern.begin(), pattern.end());
            }
        }

        msg.checksum = 0;

        // Set compression statistics
        msg.compression_ratio = result.compression_ratio;
        msg.original_point_count = static_cast<uint32_t>(result.original_size / (3 * sizeof(float)));
        msg.compressed_data_size = static_cast<uint32_t>(result.compressed_size);

        return msg;
    }

    void publishOccupiedVoxelMarkers()
    {
        if (!publish_occupied_voxel_markers_ || !marker_pub_) {
            return;
        }

        auto marker_array = createOccupiedVoxelMarkers();
        if (!marker_array.has_value()) {
            return;
        }

        marker_pub_->publish(marker_array.value());
        RCLCPP_DEBUG(this->get_logger(), "Published %zu occupied voxel markers", marker_array.value().markers.size());
    }
    
    std::optional<visualization_msgs::msg::MarkerArray> createOccupiedVoxelMarkers()
    {
        // Load point cloud
        pointcloud_compressor::PointCloud cloud;
        if (!pointcloud_compressor::PointCloudIO::loadPointCloud(input_file_, cloud)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load point cloud for marker visualization");
            return std::nullopt;
        }

        // Create voxel processor
        pointcloud_compressor::VoxelProcessor processor(
            settings_.voxel_size,
            settings_.block_size,
            settings_.min_points_threshold
        );

        // Voxelize the point cloud
        pointcloud_compressor::VoxelGrid grid;
        if (!processor.voxelizePointCloud(cloud, grid)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to voxelize point cloud");
            return std::nullopt;
        }

        // Create marker array
        visualization_msgs::msg::MarkerArray marker_array;
        auto dimensions = grid.getDimensions();
        int marker_id = 0;
        
        float origin_x, origin_y, origin_z;
        grid.getOrigin(origin_x, origin_y, origin_z);

        for (int z = 0; z < dimensions.z; ++z) {
            for (int y = 0; y < dimensions.y; ++y) {
                for (int x = 0; x < dimensions.x; ++x) {
                    if (grid.getVoxel(x, y, z)) {
                        marker_array.markers.push_back(createVoxelMarker(
                            marker_id++, x, y, z, origin_x, origin_y, origin_z
                        ));
                    }
                }
            }
        }
        
        return marker_array;
    }
    
    visualization_msgs::msg::Marker createVoxelMarker(
        int id, int x, int y, int z, 
        float origin_x, float origin_y, float origin_z)
    {
        visualization_msgs::msg::Marker marker;
        
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position at center of voxel
        marker.pose.position.x = origin_x + (x + 0.5) * settings_.voxel_size;
        marker.pose.position.y = origin_y + (y + 0.5) * settings_.voxel_size;
        marker.pose.position.z = origin_z + (z + 0.5) * settings_.voxel_size;
        marker.pose.orientation.w = 1.0;

        // Scale (slightly smaller than voxel size for visibility)
        marker.scale.x = settings_.voxel_size * 0.9;
        marker.scale.y = settings_.voxel_size * 0.9;
        marker.scale.z = settings_.voxel_size * 0.9;

        // Color (green, semi-transparent)
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;
        
        return marker;
    }
    
    void calculateAndLogPointCloudStatistics(const pointcloud_compressor::PointCloud& cloud)
    {
        if (cloud.empty()) {
            RCLCPP_WARN(this->get_logger(), "Point cloud is empty, cannot calculate statistics");
            return;
        }
        
        // Initialize min and max values
        float min_x = cloud.points[0].x;
        float max_x = cloud.points[0].x;
        float min_y = cloud.points[0].y;
        float max_y = cloud.points[0].y;
        float min_z = cloud.points[0].z;
        float max_z = cloud.points[0].z;
        
        // Find min and max values
        for (const auto& point : cloud.points) {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
            min_z = std::min(min_z, point.z);
            max_z = std::max(max_z, point.z);
        }
        
        // Calculate spatial scale (dimensions)
        float scale_x = max_x - min_x;
        float scale_y = max_y - min_y;
        float scale_z = max_z - min_z;
        
        // Log the statistics
        RCLCPP_INFO(this->get_logger(), "=== Input Point Cloud Statistics ===");
        RCLCPP_INFO(this->get_logger(), "Total points: %zu", cloud.size());
        RCLCPP_INFO(this->get_logger(), "X range: [%.3f, %.3f] meters", min_x, max_x);
        RCLCPP_INFO(this->get_logger(), "Y range: [%.3f, %.3f] meters", min_y, max_y);
        RCLCPP_INFO(this->get_logger(), "Z range: [%.3f, %.3f] meters", min_z, max_z);
        RCLCPP_INFO(this->get_logger(), "Spatial scale: %.3f x %.3f x %.3f meters", scale_x, scale_y, scale_z);
        RCLCPP_INFO(this->get_logger(), "Bounding box volume: %.3f cubic meters", scale_x * scale_y * scale_z);
        RCLCPP_INFO(this->get_logger(), "=====================================");
    }
    
    void calculateAndLogOccupancyGridSize(const pointcloud_compressor::CompressionResult& result)
    {
        // Calculate various components of occupancy grid map size
        
        // 1. Voxel Grid dimensions (get from VoxelGrid directly)
        auto dimensions = result.voxel_grid.getDimensions();
        int voxel_grid_x = dimensions.x;
        int voxel_grid_y = dimensions.y;
        int voxel_grid_z = dimensions.z;
        int total_voxels = voxel_grid_x * voxel_grid_y * voxel_grid_z;
        
        // 2. Voxel Grid raw data size (each voxel stored as uint8_t)
        size_t voxel_grid_size = total_voxels * sizeof(uint8_t);
        
        // 3. Block-based representation
        int total_blocks = result.num_blocks;
        int block_size_3d = settings_.block_size * settings_.block_size * settings_.block_size;
        
        // 4. Block indices array size
        size_t block_indices_size = result.block_indices.size() * sizeof(uint16_t);
        if (settings_.use_8bit_indices) {
            block_indices_size = result.block_indices.size() * sizeof(uint8_t);
        }
        
        // 5. Pattern dictionary size
        size_t pattern_dict_size = result.pattern_dictionary.size() * block_size_3d / 8;  // Each pattern is block_size^3 bits
        
        // 6. Metadata size (dimensions, origin, settings)
        size_t metadata_size = 
            3 * sizeof(int) +      // grid dimensions
            3 * sizeof(float) +    // grid origin
            sizeof(float) +        // voxel size
            sizeof(int) +          // block size
            sizeof(int);           // min_points_threshold
        
        // 7. Total compressed size
        size_t total_compressed_size = block_indices_size + pattern_dict_size + metadata_size;
        
        // 8. Calculate occupied voxels count
        int occupied_voxels = result.voxel_grid.getOccupiedVoxelCount();
        float occupancy_ratio = result.voxel_grid.getOccupancyRatio();
        
        // Log detailed information
        RCLCPP_INFO(this->get_logger(), "=== Occupancy Grid Map Size Analysis ===");
        RCLCPP_INFO(this->get_logger(), "Voxel Grid Dimensions: %d x %d x %d = %d total voxels", 
                    voxel_grid_x, voxel_grid_y, voxel_grid_z, total_voxels);
        RCLCPP_INFO(this->get_logger(), "Voxel Size: %.3f meters", settings_.voxel_size);
        RCLCPP_INFO(this->get_logger(), "Occupied Voxels: %d (%.2f%% occupancy)", 
                    occupied_voxels, occupancy_ratio * 100);
        RCLCPP_INFO(this->get_logger(), "Raw Voxel Grid Size: %zu bytes (%.2f KB)", 
                    voxel_grid_size, voxel_grid_size / 1024.0);
        RCLCPP_INFO(this->get_logger(), "Block Size: %d x %d x %d = %d voxels per block", 
                    settings_.block_size, settings_.block_size, settings_.block_size, block_size_3d);
        RCLCPP_INFO(this->get_logger(), "Total Blocks: %d", total_blocks);
        RCLCPP_INFO(this->get_logger(), "Block Indices Size: %zu bytes (%s)", 
                    block_indices_size, settings_.use_8bit_indices ? "8-bit" : "16-bit");
        RCLCPP_INFO(this->get_logger(), "Pattern Dictionary: %zu patterns, %zu bytes (%.2f KB)", 
                    result.num_unique_patterns, pattern_dict_size, pattern_dict_size / 1024.0);
        RCLCPP_INFO(this->get_logger(), "Metadata Size: %zu bytes", metadata_size);
        RCLCPP_INFO(this->get_logger(), "Total Compressed Size: %zu bytes (%.2f KB)", 
                    total_compressed_size, total_compressed_size / 1024.0);
        RCLCPP_INFO(this->get_logger(), "Memory Reduction: %.2f%% (from %.2f KB to %.2f KB)", 
                    (1.0 - (double)total_compressed_size / voxel_grid_size) * 100,
                    voxel_grid_size / 1024.0, total_compressed_size / 1024.0);
        RCLCPP_INFO(this->get_logger(), "========================================");
    }
    
    void cleanupTempFiles(const std::string& prefix)
    {
        std::vector<std::string> extensions = {"_dict.bin", "_indices.bin", "_meta.bin"};
        for (const auto& ext : extensions) {
            std::string filename = prefix + ext;
            if (std::filesystem::exists(filename)) {
                std::filesystem::remove(filename);
            }
        }
    }

    // Member variables
    std::unique_ptr<pointcloud_compressor::PointCloudCompressor> compressor_;
    rclcpp::Publisher<pointcloud_compressor::msg::PatternDictionary>::SharedPtr pattern_dict_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::string input_file_;
    bool compressed_published_;
    bool publish_occupied_voxel_markers_;
    bool publish_once_;
    int publish_interval_ms_;
    pointcloud_compressor::CompressionSettings settings_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<PointCloudCompressorNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pointcloud_compressor_node"),
                     "Exception in main: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}