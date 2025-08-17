#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <filesystem>
#include <chrono>

#include "pointcloud_compressor/msg/compressed_point_cloud.hpp"
#include "pointcloud_compressor/msg/compression_settings.hpp"
#include "pointcloud_compressor/msg/pattern_dictionary.hpp"
#include "pointcloud_compressor/core/PointCloudCompressor.hpp"

class PointCloudCompressorNode : public rclcpp::Node
{
public:
    PointCloudCompressorNode() : Node("pointcloud_compressor_node"), compressed_published_(false)
    {
        // Declare parameters
        this->declare_parameter("input_pcd_file", "");
        this->declare_parameter("voxel_size", 0.01);
        this->declare_parameter("block_size", 8);
        this->declare_parameter("use_8bit_indices", false);
        this->declare_parameter("publish_once", true);
        this->declare_parameter("publish_interval_ms", 1000);

        // Get parameters
        input_pcd_file_ = this->get_parameter("input_pcd_file").as_string();
        
        pointcloud_compressor::CompressionSettings settings;
        settings.voxel_size = this->get_parameter("voxel_size").as_double();
        settings.block_size = this->get_parameter("block_size").as_int();
        settings.use_8bit_indices = this->get_parameter("use_8bit_indices").as_bool();
        
        bool publish_once = this->get_parameter("publish_once").as_bool();
        int publish_interval_ms = this->get_parameter("publish_interval_ms").as_int();

        // Initialize compressor
        compressor_ = std::make_unique<pointcloud_compressor::PointCloudCompressor>(settings);

        // Create publisher
        compressed_pub_ = this->create_publisher<pointcloud_compressor::msg::CompressedPointCloud>(
            "compressed_pointcloud", 10);

        // Validate input file
        if (input_pcd_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No input PCD file specified. Use parameter 'input_pcd_file'");
            return;
        }

        if (!compressor_->validateInputFile(input_pcd_file_)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid input PCD file: %s", input_pcd_file_.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "PointCloud Compressor Node initialized");
        RCLCPP_INFO(this->get_logger(), "Input file: %s", input_pcd_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "Voxel size: %.3f", settings.voxel_size);
        RCLCPP_INFO(this->get_logger(), "Block size: %d", settings.block_size);
        RCLCPP_INFO(this->get_logger(), "Use 8-bit indices: %s", settings.use_8bit_indices ? "true" : "false");

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

        RCLCPP_INFO(this->get_logger(), "Starting compression of: %s", input_pcd_file_.c_str());

        // Perform compression
        auto start_time = std::chrono::high_resolution_clock::now();
        std::string temp_prefix = "/tmp/compressed_" + std::to_string(this->now().nanoseconds());
        auto compression_result = compressor_->compress(input_pcd_file_, temp_prefix);
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

        // Create and populate compressed message
        auto msg = createCompressedMessage(compression_result, 
                                         static_cast<double>(compression_duration.count()) / 1000.0);

        // Publish the message
        compressed_pub_->publish(msg);
        compressed_published_ = true;

        RCLCPP_INFO(this->get_logger(), "Published compressed point cloud data");

        // Clean up temporary files
        cleanupTempFiles(temp_prefix);
    }

    pointcloud_compressor::msg::CompressedPointCloud createCompressedMessage(
        const pointcloud_compressor::CompressionResult& result,
        double compression_time_seconds)
    {
        auto msg = pointcloud_compressor::msg::CompressedPointCloud();

        // Set header
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";

        // Set original data information (simplified for this demo)
        msg.original_point_count = static_cast<uint32_t>(result.original_size / (3 * sizeof(float)));
        msg.original_data_size = static_cast<uint32_t>(result.original_size);

        // Set voxel grid information (would need to get from actual voxel grid)
        msg.voxel_grid_dimensions.x = 0.0;  // Would be filled with actual data
        msg.voxel_grid_dimensions.y = 0.0;
        msg.voxel_grid_dimensions.z = 0.0;
        msg.voxel_grid_origin.x = 0.0;
        msg.voxel_grid_origin.y = 0.0;
        msg.voxel_grid_origin.z = 0.0;

        // Set block information
        msg.total_blocks = static_cast<uint32_t>(result.num_blocks);
        msg.blocks_x = 0;  // Would be calculated from actual data
        msg.blocks_y = 0;
        msg.blocks_z = 0;

        // Set compression settings
        auto settings = compressor_->getSettings();
        msg.compression_settings.voxel_size = settings.voxel_size;
        msg.compression_settings.block_size = static_cast<uint32_t>(settings.block_size);
        msg.compression_settings.use_8bit_indices = settings.use_8bit_indices;
        msg.compression_settings.algorithm_version = "1.0.0";

        // Set pattern dictionary (simplified - would need actual dictionary data)
        msg.pattern_dictionary.num_patterns = static_cast<uint32_t>(result.num_unique_patterns);
        msg.pattern_dictionary.pattern_size_bytes = 64;  // Example value
        msg.pattern_dictionary.checksum = 0;  // Would calculate actual CRC32

        // Set compression statistics
        msg.compression_ratio = result.compression_ratio;
        msg.compressed_data_size = static_cast<uint32_t>(result.compressed_size);
        msg.compression_time_seconds = compression_time_seconds;
        msg.unique_patterns_count = static_cast<uint32_t>(result.num_unique_patterns);
        msg.reconstruction_error = 0.0f;  // Would calculate if available

        return msg;
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
    rclcpp::Publisher<pointcloud_compressor::msg::CompressedPointCloud>::SharedPtr compressed_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string input_pcd_file_;
    bool compressed_published_;
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