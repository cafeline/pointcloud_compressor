// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <filesystem>
#include <iomanip>
#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/core/VoxelProcessor.hpp"
#include "pointcloud_compressor/io/PcdIO.hpp"
#include "pointcloud_compressor/io/PlyIO.hpp"

namespace pointcloud_compressor {

class BlockSizeOptimizerNode : public rclcpp::Node {
public:
    BlockSizeOptimizerNode() : Node("block_size_optimizer") {
        // Declare parameters with defaults
        this->declare_parameter<std::string>("input_file", "");
        this->declare_parameter<int>("min_block_size", 4);
        this->declare_parameter<int>("max_block_size", 32);
        this->declare_parameter<int>("step_size", 1);
        this->declare_parameter<double>("voxel_size", 0.01);
        this->declare_parameter<bool>("verbose", false);
        this->declare_parameter<bool>("auto_compress", false);
        this->declare_parameter<std::string>("output_prefix", "");
        this->declare_parameter<bool>("run_once", true);
        
        // Publishers for results
        result_pub_ = this->create_publisher<std_msgs::msg::String>("optimization_result", 10);
        optimal_block_size_pub_ = this->create_publisher<std_msgs::msg::Int32>("optimal_block_size", 10);
        compression_ratio_pub_ = this->create_publisher<std_msgs::msg::Float32>("best_compression_ratio", 10);
        
        // Get parameters
        input_file_ = this->get_parameter("input_file").as_string();
        min_block_size_ = this->get_parameter("min_block_size").as_int();
        max_block_size_ = this->get_parameter("max_block_size").as_int();
        step_size_ = this->get_parameter("step_size").as_int();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        verbose_ = this->get_parameter("verbose").as_bool();
        auto_compress_ = this->get_parameter("auto_compress").as_bool();
        output_prefix_ = this->get_parameter("output_prefix").as_string();
        run_once_ = this->get_parameter("run_once").as_bool();
        
        // Validate input file
        if (input_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No input file specified!");
            rclcpp::shutdown();
            return;
        }
        
        if (!std::filesystem::exists(input_file_)) {
            RCLCPP_ERROR(this->get_logger(), "Input file does not exist: %s", input_file_.c_str());
            rclcpp::shutdown();
            return;
        }
        
        // Initialize compressor
        CompressionSettings settings(voxel_size_, 8);
        compressor_ = std::make_unique<PointCloudCompressor>(settings);
        
        RCLCPP_INFO(this->get_logger(), 
                    "Block Size Optimizer Node initialized");
        RCLCPP_INFO(this->get_logger(), 
                    "Input file: %s", input_file_.c_str());
        RCLCPP_INFO(this->get_logger(), 
                    "Block size range: %d to %d (step: %d)", 
                    min_block_size_, max_block_size_, step_size_);
        RCLCPP_INFO(this->get_logger(), 
                    "Voxel size: %.3f", voxel_size_);
        
        // If run_once is true, execute immediately
        if (run_once_) {
            executeOptimization();
        } else {
            // Otherwise, create a service for on-demand optimization
            optimization_service_ = this->create_service<std_srvs::srv::Trigger>(
                "run_optimization",
                std::bind(&BlockSizeOptimizerNode::handleOptimizationRequest, this,
                         std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), 
                       "Service 'run_optimization' is ready. Call it to start optimization.");
        }
    }
    
private:
    void executeOptimization() {
        RCLCPP_INFO(this->get_logger(), "Starting block size optimization...");
        
        auto start_time = this->now();
        
        // Run optimization
        auto result = compressor_->findOptimalBlockSize(
            input_file_, min_block_size_, max_block_size_, step_size_, verbose_);
        
        auto end_time = this->now();
        auto duration = (end_time - start_time).seconds();
        
        // Check if optimization was successful
        if (result.optimal_block_size < 0) {
            RCLCPP_ERROR(this->get_logger(), "Optimization failed!");
            publishFailure();
            if (run_once_) {
                // Schedule shutdown after a short delay
                auto timer = this->create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() {
                        rclcpp::shutdown();
                    });
            }
            return;
        }
        
        // Log results
        RCLCPP_INFO(this->get_logger(), 
                   "=== Optimization Results ===");
        RCLCPP_INFO(this->get_logger(), 
                   "Optimal block size: %d", result.optimal_block_size);
        RCLCPP_INFO(this->get_logger(), 
                   "Best compression ratio: %.6f", result.best_compression_ratio);
        RCLCPP_INFO(this->get_logger(), 
                   "Optimization time: %.2f ms (ROS timer: %.2f s)", 
                   result.optimization_time_ms, duration);
        
        // Store last result for analysis
        last_result_ = result;
        
        // Calculate and display 1 byte/voxel comparison
        // Note: This requires grid information, so we'll estimate based on the input file
        calculateAndDisplay1ByteComparison();
        
        // Calculate grid info for 1 byte/voxel comparison
        PointCloud temp_cloud;
        std::string ext = input_file_.substr(input_file_.find_last_of('.'));
        if (ext == ".ply") {
            PlyIO::readPlyFile(input_file_, temp_cloud);
        } else {
            PcdIO::readPcdFile(input_file_, temp_cloud);
        }
        
        VoxelProcessor temp_processor(voxel_size_, 8);
        VoxelGrid temp_grid;
        temp_processor.voxelizePointCloud(temp_cloud, temp_grid);
        VoxelCoord dims = temp_grid.getDimensions();
        size_t total_voxels = static_cast<size_t>(dims.x) * dims.y * dims.z;
        size_t one_byte_per_voxel_size = total_voxels * 1;
        size_t original_point_cloud_size = temp_cloud.points.size() * sizeof(Point3D);
        
        // Log all tested results with 1 byte/voxel comparison
        RCLCPP_INFO(this->get_logger(), "All tested block sizes (ratio vs 1 byte/voxel):");
        for (const auto& [block_size, ratio] : result.tested_results) {
            // Calculate actual compressed size from ratio
            size_t compressed_size = static_cast<size_t>(ratio * original_point_cloud_size);
            float ratio_vs_one_byte = static_cast<float>(compressed_size) / static_cast<float>(one_byte_per_voxel_size);
            
            if (block_size == result.optimal_block_size) {
                RCLCPP_INFO(this->get_logger(), 
                           "  Block size %d: ratio = %.6f (%.1fx smaller than 1 byte/voxel) <- BEST", 
                           block_size, ratio_vs_one_byte, 1.0f / ratio_vs_one_byte);
            } else {
                RCLCPP_INFO(this->get_logger(), 
                           "  Block size %d: ratio = %.6f (%.1fx smaller than 1 byte/voxel)", 
                           block_size, ratio_vs_one_byte, 1.0f / ratio_vs_one_byte);
            }
        }
        
        // Publish results
        publishResults(result);
        
        // Auto compress if requested
        if (auto_compress_) {
            performCompression(result.optimal_block_size);
        }
        
        // Shutdown if run_once mode
        if (run_once_) {
            RCLCPP_INFO(this->get_logger(), "Optimization complete. Shutting down node.");
            // Schedule shutdown after a short delay to allow cleanup
            auto timer = this->create_wall_timer(
                std::chrono::milliseconds(100),
                [this]() {
                    rclcpp::shutdown();
                });
        }
    }
    
    void handleOptimizationRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        (void)request; // Unused
        
        try {
            executeOptimization();
            response->success = true;
            response->message = "Optimization completed successfully";
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Optimization failed: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Optimization failed: %s", e.what());
        }
    }
    
    void publishResults(const BlockSizeOptimizationResult& result) {
        // Publish detailed result as JSON string
        std::stringstream ss;
        ss << "{\n";
        ss << "  \"optimal_block_size\": " << result.optimal_block_size << ",\n";
        ss << "  \"best_compression_ratio\": " << std::fixed << std::setprecision(6) 
           << result.best_compression_ratio << ",\n";
        ss << "  \"optimization_time_ms\": " << std::fixed << std::setprecision(2) 
           << result.optimization_time_ms << ",\n";
        ss << "  \"tested_results\": {\n";
        
        bool first = true;
        for (const auto& [block_size, ratio] : result.tested_results) {
            if (!first) ss << ",\n";
            ss << "    \"" << block_size << "\": " << std::fixed << std::setprecision(6) << ratio;
            first = false;
        }
        ss << "\n  }\n";
        ss << "}";
        
        auto msg = std_msgs::msg::String();
        msg.data = ss.str();
        result_pub_->publish(msg);
        
        // Publish individual values
        auto block_size_msg = std_msgs::msg::Int32();
        block_size_msg.data = result.optimal_block_size;
        optimal_block_size_pub_->publish(block_size_msg);
        
        auto ratio_msg = std_msgs::msg::Float32();
        ratio_msg.data = result.best_compression_ratio;
        compression_ratio_pub_->publish(ratio_msg);
    }
    
    void publishFailure() {
        auto msg = std_msgs::msg::String();
        msg.data = "{\"error\": \"Optimization failed\"}";
        result_pub_->publish(msg);
        
        auto block_size_msg = std_msgs::msg::Int32();
        block_size_msg.data = -1;
        optimal_block_size_pub_->publish(block_size_msg);
        
        auto ratio_msg = std_msgs::msg::Float32();
        ratio_msg.data = -1.0f;
        compression_ratio_pub_->publish(ratio_msg);
    }
    
    void performCompression(int optimal_block_size) {
        RCLCPP_INFO(this->get_logger(), 
                   "=== Compressing with optimal block size %d ===", 
                   optimal_block_size);
        
        // Update settings with optimal block size
        CompressionSettings settings(voxel_size_, optimal_block_size);
        compressor_->updateSettings(settings);
        
        // Determine output prefix
        std::string prefix = output_prefix_;
        if (prefix.empty()) {
            prefix = input_file_;
            size_t dot_pos = prefix.find_last_of('.');
            if (dot_pos != std::string::npos) {
                prefix = prefix.substr(0, dot_pos);
            }
            prefix += "_optimized";
        }
        
        // Perform compression
        auto compress_result = compressor_->compress(input_file_, prefix);
        
        if (compress_result.success) {
            RCLCPP_INFO(this->get_logger(), "Compression successful!");
            RCLCPP_INFO(this->get_logger(), 
                       "Original size: %zu bytes", compress_result.original_size);
            RCLCPP_INFO(this->get_logger(), 
                       "Compressed size: %zu bytes", compress_result.compressed_size);
            RCLCPP_INFO(this->get_logger(), 
                       "Compression ratio: %.6f", compress_result.compression_ratio);
            RCLCPP_INFO(this->get_logger(), 
                       "Output files: %s.*", prefix.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                        "Compression failed: %s", compress_result.error_message.c_str());
        }
    }
    
    // Parameters
    std::string input_file_;
    int min_block_size_;
    int max_block_size_;
    int step_size_;
    double voxel_size_;
    bool verbose_;
    bool auto_compress_;
    std::string output_prefix_;
    bool run_once_;
    
    // ROS interfaces
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr optimal_block_size_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr compression_ratio_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr optimization_service_;
    
    // Compressor
    std::unique_ptr<PointCloudCompressor> compressor_;
    
    // Store last optimization result for analysis
    BlockSizeOptimizationResult last_result_;
    
    void calculateAndDisplay1ByteComparison() {
        // Quick voxelization to get grid dimensions
        PointCloud cloud;
        
        // Load point cloud based on file extension
        std::string ext = input_file_.substr(input_file_.find_last_of('.'));
        if (ext == ".ply") {
            PlyIO::readPlyFile(input_file_, cloud);
        } else {
            PcdIO::readPcdFile(input_file_, cloud);
        }
        
        CompressionSettings temp_settings(voxel_size_, 8);
        VoxelProcessor temp_processor(voxel_size_, 8);
        VoxelGrid grid;
        
        if (temp_processor.voxelizePointCloud(cloud, grid)) {
            VoxelCoord dims = grid.getDimensions();
            size_t total_voxels = static_cast<size_t>(dims.x) * dims.y * dims.z;
            size_t occupied_voxels = grid.getOccupiedVoxelCount();
            
            // Calculate sizes
            size_t one_byte_per_voxel_size = total_voxels * 1;  // 1 byte per voxel for ALL voxels
            size_t original_point_cloud_size = cloud.points.size() * sizeof(Point3D);
            
            // Calculate real-world dimensions
            float world_size_x = dims.x * voxel_size_;
            float world_size_y = dims.y * voxel_size_;
            float world_size_z = dims.z * voxel_size_;
            
            // Calculate 1 bit/voxel size
            size_t one_bit_per_voxel_size = (total_voxels + 7) / 8;
            
            RCLCPP_INFO(this->get_logger(), 
                       "\n============ Final Summary ============");
            RCLCPP_INFO(this->get_logger(), 
                       "Real-world scale: %.1fm x %.1fm x %.1fm", 
                       world_size_x, world_size_y, world_size_z);
            RCLCPP_INFO(this->get_logger(), 
                       "Total voxels: %zu", total_voxels);
            RCLCPP_INFO(this->get_logger(), 
                       "Occupied voxels: %zu (%.2f%%)", 
                       occupied_voxels, 
                       (100.0f * occupied_voxels / total_voxels));
            RCLCPP_INFO(this->get_logger(), 
                       "1 byte/voxel map size: %zu bytes", 
                       one_byte_per_voxel_size);
            RCLCPP_INFO(this->get_logger(), 
                       "1 bit/voxel map size: %zu bytes", 
                       one_bit_per_voxel_size);
            
            // Calculate our best compression vs 1 byte/voxel
            if (!last_result_.tested_results.empty()) {
                // Find best compression size (need to recalculate from ratio)
                size_t best_compressed_size = static_cast<size_t>(
                    last_result_.best_compression_ratio * original_point_cloud_size);
                
                // Estimate codebook and index sizes based on optimal block size
                int optimal_bs = last_result_.optimal_block_size;
                size_t pattern_bytes = (optimal_bs * optimal_bs * optimal_bs + 7) / 8;
                
                // Estimate from grid
                int x_blocks = (dims.x + optimal_bs - 1) / optimal_bs;
                int y_blocks = (dims.y + optimal_bs - 1) / optimal_bs;
                int z_blocks = (dims.z + optimal_bs - 1) / optimal_bs;
                size_t total_blocks = x_blocks * y_blocks * z_blocks;
                
                // Rough estimate of codebook vs index split (typical: 20% codebook, 80% index)
                size_t estimated_codebook = best_compressed_size * 0.2;
                size_t estimated_index = best_compressed_size * 0.8;
                
                float compression_vs_one_byte = static_cast<float>(best_compressed_size) / 
                                               static_cast<float>(one_byte_per_voxel_size);
                float compression_vs_one_bit = static_cast<float>(best_compressed_size) / 
                                              static_cast<float>(one_bit_per_voxel_size);
                
                RCLCPP_INFO(this->get_logger(), 
                           "Compressed map (block_size=%d):", optimal_bs);
                RCLCPP_INFO(this->get_logger(), 
                           "  - Codebook: ~%zu bytes", estimated_codebook);
                RCLCPP_INFO(this->get_logger(), 
                           "  - Index map: ~%zu bytes", estimated_index);
                RCLCPP_INFO(this->get_logger(), 
                           "  - Total: %zu bytes", best_compressed_size);
                RCLCPP_INFO(this->get_logger(), 
                           "Compression ratio (vs 1 byte/voxel): %.6f (%.1fx smaller)", 
                           compression_vs_one_byte, 
                           1.0f / compression_vs_one_byte);
                RCLCPP_INFO(this->get_logger(), 
                           "Compression ratio (vs 1 bit/voxel): %.6f (%.1fx smaller)", 
                           compression_vs_one_bit, 
                           1.0f / compression_vs_one_bit);
                RCLCPP_INFO(this->get_logger(), 
                           "========================================");
            }
        }
    }
};

} // namespace pointcloud_compressor

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pointcloud_compressor::BlockSizeOptimizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}