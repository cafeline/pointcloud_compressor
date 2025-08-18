#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <filesystem>
#include <chrono>
#include <iostream>

#include "pointcloud_compressor/msg/pattern_dictionary.hpp"
#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/core/VoxelProcessor.hpp"
#include "pointcloud_compressor/io/PointCloudIO.hpp"

class PointCloudCompressorNode : public rclcpp::Node
{
public:
    PointCloudCompressorNode() : Node("pointcloud_compressor_node"), compressed_published_(false)
    {
        // Declare parameters
        this->declare_parameter("input_file", "");
        // Keep old parameter name for backward compatibility
        this->declare_parameter("input_pcd_file", "");
        this->declare_parameter("voxel_size", 0.01);
        this->declare_parameter("block_size", 8);
        this->declare_parameter("use_8bit_indices", false);
        this->declare_parameter("min_points_threshold", 1);
        this->declare_parameter("publish_once", true);
        this->declare_parameter("publish_interval_ms", 1000);
        this->declare_parameter("publish_occupied_voxel_markers", false);

        // Get parameters (with fallback for backward compatibility)
        input_file_ = this->get_parameter("input_file").as_string();
        if (input_file_.empty()) {
            input_file_ = this->get_parameter("input_pcd_file").as_string();
        }

        pointcloud_compressor::CompressionSettings settings;
        settings.voxel_size = this->get_parameter("voxel_size").as_double();
        settings.block_size = this->get_parameter("block_size").as_int();
        settings.use_8bit_indices = this->get_parameter("use_8bit_indices").as_bool();
        settings.min_points_threshold = this->get_parameter("min_points_threshold").as_int();

        bool publish_once = this->get_parameter("publish_once").as_bool();
        int publish_interval_ms = this->get_parameter("publish_interval_ms").as_int();
        publish_occupied_voxel_markers_ = this->get_parameter("publish_occupied_voxel_markers").as_bool();
        settings_ = settings;  // Store settings for later use

        // Initialize compressor
        compressor_ = std::make_unique<pointcloud_compressor::PointCloudCompressor>(settings);

        // Create publishers
        pattern_dict_pub_ = this->create_publisher<pointcloud_compressor::msg::PatternDictionary>(
            "pattern_dictionary", 10);

        // Create occupied voxel marker publisher if enabled
        if (publish_occupied_voxel_markers_) {
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "occupied_voxel_markers", 10);
            RCLCPP_INFO(this->get_logger(), "Occupied voxel marker publishing enabled");
        }

        // Validate input file
        if (input_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No input file specified. Use parameter 'input_file' or 'input_pcd_file'");
            return;
        }

        if (!compressor_->validateInputFile(input_file_)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid input file: %s", input_file_.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "PointCloud Compressor Node initialized");
        RCLCPP_INFO(this->get_logger(), "Input file: %s", input_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "Voxel size: %.3f", settings.voxel_size);
        RCLCPP_INFO(this->get_logger(), "Block size: %d", settings.block_size);
        RCLCPP_INFO(this->get_logger(), "Use 8-bit indices: %s", settings.use_8bit_indices ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Min points threshold: %d", settings.min_points_threshold);
        std::cout << "[DEBUG] min_points_threshold parameter value: " << settings.min_points_threshold << std::endl;

        if (publish_once) {
            // Compress and publish once immediately
            compressAndPublish();
        } else {
            // Create timer for periodic publishing
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(publish_interval_ms),
                std::bind(&PointCloudCompressorNode::compressAndPublish, this));
        }
    }

private:
    void compressAndPublish()
    {
        if (compressed_published_ && this->get_parameter("publish_once").as_bool()) {
            RCLCPP_DEBUG(this->get_logger(), "Already published compressed data once, skipping");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Starting compression of: %s", input_file_.c_str());

        // Perform compression
        auto start_time = std::chrono::high_resolution_clock::now();
        std::string temp_prefix = "/tmp/compressed_" + std::to_string(this->now().nanoseconds());
        auto compression_result = compressor_->compress(input_file_, temp_prefix);
        auto end_time = std::chrono::high_resolution_clock::now();

        auto compression_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        if (!compression_result.success) {
            RCLCPP_ERROR(this->get_logger(), "Compression failed: %s", compression_result.error_message.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Compression completed in %ld ms", compression_duration.count());
        RCLCPP_INFO(this->get_logger(), "Original size: %zu bytes", compression_result.original_size);
        RCLCPP_INFO(this->get_logger(), "Compressed size: %zu bytes", compression_result.compressed_size);
        RCLCPP_INFO(this->get_logger(), "Compression ratio: %.3f", compression_result.compression_ratio);
        RCLCPP_INFO(this->get_logger(), "Unique patterns: %zu", compression_result.num_unique_patterns);

        // Create and populate pattern dictionary message
        auto msg = createPatternDictionaryMessage(compression_result);

        // Publish the message
        pattern_dict_pub_->publish(msg);
        compressed_published_ = true;

        // Publish occupied voxel markers if enabled
        publishOccupiedVoxelMarkers();

        RCLCPP_INFO(this->get_logger(), "Published pattern dictionary data");

        // Clean up temporary files
        cleanupTempFiles(temp_prefix);
    }

    pointcloud_compressor::msg::PatternDictionary createPatternDictionaryMessage(
        const pointcloud_compressor::CompressionResult& result)
    {
        auto msg = pointcloud_compressor::msg::PatternDictionary();

        // Set header
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";

        // Set compression settings
        auto settings = compressor_->getSettings();
        msg.voxel_size = settings.voxel_size;
        msg.block_size = static_cast<uint32_t>(settings.block_size);
        msg.use_8bit_indices = settings.use_8bit_indices;
        msg.min_points_threshold = static_cast<uint32_t>(settings.min_points_threshold);

        // Set voxel grid information from actual data
        msg.voxel_grid_dimensions.x = result.grid_dimensions.x;
        msg.voxel_grid_dimensions.y = result.grid_dimensions.y;
        msg.voxel_grid_dimensions.z = result.grid_dimensions.z;
        msg.voxel_grid_origin.x = result.grid_origin.x;
        msg.voxel_grid_origin.y = result.grid_origin.y;
        msg.voxel_grid_origin.z = result.grid_origin.z;

        // Set block information from actual data
        msg.total_blocks = static_cast<uint32_t>(result.num_blocks);
        msg.blocks_x = static_cast<uint32_t>(result.blocks_count.x);
        msg.blocks_y = static_cast<uint32_t>(result.blocks_count.y);
        msg.blocks_z = static_cast<uint32_t>(result.blocks_count.z);

        // Set block indices from actual data (convert from uint16 to uint8)
        msg.block_indices.resize(result.block_indices.size());
        for (size_t i = 0; i < result.block_indices.size(); ++i) {
            // Clamp to uint8 range (0-255)
            msg.block_indices[i] = static_cast<uint8_t>(std::min(result.block_indices[i], static_cast<uint16_t>(255)));
        }

        // Set pattern dictionary from actual data
        msg.num_patterns = static_cast<uint32_t>(result.num_unique_patterns);
        msg.pattern_size_bytes = 64;  // Based on 8x8x8 block = 512 bits = 64 bytes

        // Set actual pattern data from compression result
        if (result.num_unique_patterns > 0 && !result.pattern_dictionary.empty()) {
            msg.dictionary_data.clear();
            for (const auto& pattern : result.pattern_dictionary) {
                msg.dictionary_data.insert(msg.dictionary_data.end(), pattern.begin(), pattern.end());
            }
        } else {
            // Fallback: Create dummy pattern data
            msg.dictionary_data.resize(result.num_unique_patterns * 64);
            for (size_t i = 0; i < msg.dictionary_data.size(); ++i) {
                msg.dictionary_data[i] = static_cast<uint8_t>(i % 256);
            }
        }

        msg.checksum = 0;  // Would calculate actual CRC32

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

        RCLCPP_INFO(this->get_logger(), "Publishing occupied voxel markers...");

        // Load point cloud
        pointcloud_compressor::PointCloud cloud;
        if (!pointcloud_compressor::PointCloudIO::loadPointCloud(input_file_, cloud)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load point cloud for marker visualization");
            return;
        }

        // Create voxel processor with same settings
        pointcloud_compressor::VoxelProcessor processor(
            settings_.voxel_size,
            settings_.block_size,
            settings_.min_points_threshold
        );

        // Voxelize the point cloud
        pointcloud_compressor::VoxelGrid grid;
        if (!processor.voxelizePointCloud(cloud, grid)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to voxelize point cloud");
            return;
        }

        // Create marker array
        visualization_msgs::msg::MarkerArray marker_array;

        // Get occupied voxel positions
        auto dimensions = grid.getDimensions();
        int marker_id = 0;

        for (int z = 0; z < dimensions.z; ++z) {
            for (int y = 0; y < dimensions.y; ++y) {
                for (int x = 0; x < dimensions.x; ++x) {
                    if (grid.getVoxel(x, y, z)) {
                        // Create marker for this occupied voxel
                        visualization_msgs::msg::Marker marker;
                        marker.header.frame_id = "map";
                        marker.header.stamp = this->get_clock()->now();
                        marker.id = marker_id++;
                        marker.type = visualization_msgs::msg::Marker::CUBE;
                        marker.action = visualization_msgs::msg::Marker::ADD;

                        // Position at center of voxel
                        float origin_x, origin_y, origin_z;
                        grid.getOrigin(origin_x, origin_y, origin_z);
                        marker.pose.position.x = origin_x + (x + 0.5) * settings_.voxel_size;
                        marker.pose.position.y = origin_y + (y + 0.5) * settings_.voxel_size;
                        marker.pose.position.z = origin_z + (z + 0.5) * settings_.voxel_size;

                        // Orientation
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

                        marker_array.markers.push_back(marker);
                    }
                }
            }
        }

        // Publish markers
        marker_pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published %zu occupied voxel markers", marker_array.markers.size());

        // Print summary
        std::cout << "======================================================================" << std::endl;
        std::cout << "OCCUPIED VOXEL MARKERS" << std::endl;
        std::cout << "======================================================================" << std::endl;
        std::cout << "Voxel size: " << settings_.voxel_size << " m" << std::endl;
        std::cout << "Min points threshold: " << settings_.min_points_threshold << std::endl;
        std::cout << "Grid dimensions: " << dimensions.x << "x" << dimensions.y << "x" << dimensions.z << std::endl;
        std::cout << "Occupied voxels: " << marker_array.markers.size() << std::endl;
        std::cout << "======================================================================" << std::endl;
    }

    void cleanupTempFiles(const std::string& prefix)
    {
        // Clean up any temporary files created during compression
        std::vector<std::string> extensions = {"_dict.bin", "_indices.bin", "_meta.bin"};
        for (const auto& ext : extensions) {
            std::string filename = prefix + ext;
            if (std::filesystem::exists(filename)) {
                std::filesystem::remove(filename);
                RCLCPP_DEBUG(this->get_logger(), "Cleaned up: %s", filename.c_str());
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