// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <optional>
#include "vq_occupancy_compressor/config/CompressorConfig.hpp"
#include "vq_occupancy_compressor/config/ConfigTransforms.hpp"
#include "vq_occupancy_compressor/core/VqOccupancyCompressor.hpp"
#include "vq_occupancy_compressor/ros/BlockSizeOptimizerSummary.hpp"

namespace vq_occupancy_compressor {

class BlockSizeOptimizerNode : public rclcpp::Node {
public:
    BlockSizeOptimizerNode() : Node("block_size_optimizer") {
        
        this->declare_parameter<std::string>("input_file", "");
        this->declare_parameter<int>("min_block_size", 4);
        this->declare_parameter<int>("max_block_size", 32);
        this->declare_parameter<int>("step_size", 1);
        this->declare_parameter<double>("voxel_size", 0.01);
        this->declare_parameter<bool>("verbose", false);
        this->declare_parameter<bool>("run_once", true);
        
        
        result_pub_ = this->create_publisher<std_msgs::msg::String>("optimization_result", 10);
        optimal_block_size_pub_ = this->create_publisher<std_msgs::msg::Int32>("optimal_block_size", 10);
        compression_ratio_pub_ = this->create_publisher<std_msgs::msg::Float32>("best_compression_ratio", 10);
        
        
        input_file_ = this->get_parameter("input_file").as_string();
        min_block_size_ = this->get_parameter("min_block_size").as_int();
        max_block_size_ = this->get_parameter("max_block_size").as_int();
        step_size_ = this->get_parameter("step_size").as_int();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        verbose_ = this->get_parameter("verbose").as_bool();
        run_once_ = this->get_parameter("run_once").as_bool();


        auto base_config = buildBaseConfig();
        compression_setup_ = vq_occupancy_compressor::config::buildCompressionSetup(base_config);

        auto errors = vq_occupancy_compressor::config::validateCompressionSetup(compression_setup_);
        if (!errors.empty()) {
            for (const auto& err : errors) {
                RCLCPP_ERROR(this->get_logger(), "%s", err.c_str());
            }
            rclcpp::shutdown();
            return;
        }

        
        compressor_ = std::make_unique<VqOccupancyCompressor>(compression_setup_.settings);
        
        RCLCPP_INFO(this->get_logger(), 
                    "Block Size Optimizer Node initialized");
        RCLCPP_INFO(this->get_logger(), 
                    "Input file: %s", input_file_.c_str());
        RCLCPP_INFO(this->get_logger(), 
                    "Block size range: %d to %d (step: %d)", 
                    min_block_size_, max_block_size_, step_size_);
        RCLCPP_INFO(this->get_logger(), 
                    "Voxel size: %.3f", voxel_size_);
        
        
        if (run_once_) {
            executeOptimization();
        } else {
            
            optimization_service_ = this->create_service<std_srvs::srv::Trigger>(
                "run_optimization",
                std::bind(&BlockSizeOptimizerNode::handleOptimizationRequest, this,
                         std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), 
                       "Service 'run_optimization' is ready. Call it to start optimization.");
        }
    }
    
private:
    config::CompressorConfig buildBaseConfig() const {
        config::CompressorConfig config;
        config.input_file = input_file_;
        config.voxel_size = voxel_size_;
        config.block_size = 8;
        config.save_hdf5 = false;
        config.save_raw_hdf5 = false;
        return config;
    }

    void executeOptimization() {
        RCLCPP_INFO(this->get_logger(), "Starting block size optimization...");
        
        auto start_time = this->now();
        
        
        auto result = compressor_->findOptimalBlockSize(
            input_file_, min_block_size_, max_block_size_, step_size_, verbose_);
        
        auto end_time = this->now();
        (void)start_time;
        (void)end_time;
        
        
        if (result.optimal_block_size < 0) {
            RCLCPP_ERROR(this->get_logger(), "Optimization failed!");
            publishFailure();
            if (run_once_) {
                
                auto timer = this->create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() {
                        rclcpp::shutdown();
                    });
            }
            return;
        }
        
        const CompressionResult* compression_result = nullptr;
        if (result.optimal_block_size > 0) {
            compression_result = performCompression(result.optimal_block_size);
        }

        if (!compression_result) {
            return;
        }

        displayCompressionSummary(*compression_result, result.optimal_block_size);
        publishResults(result.optimal_block_size, *compression_result);
        
        
        if (run_once_) {
            RCLCPP_INFO(this->get_logger(), "Optimization complete. Shutting down node.");
            
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
        
        (void)request; 
        
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
    
    void publishResults(int optimal_block_size, const CompressionResult& compression_result) {
        const auto summary_json =
            vq_occupancy_compressor::ros::buildOptimizationSummaryJson(optimal_block_size,
                                                                       compression_result);

        auto msg = std_msgs::msg::String();
        msg.data = summary_json;
        result_pub_->publish(msg);
        
        
        auto block_size_msg = std_msgs::msg::Int32();
        block_size_msg.data = optimal_block_size;
        optimal_block_size_pub_->publish(block_size_msg);
        
        auto ratio_msg = std_msgs::msg::Float32();
        ratio_msg.data = compression_result.compression_ratio;
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
    
    const CompressionResult* performCompression(int optimal_block_size) {
        RCLCPP_INFO(this->get_logger(),
                   "=== Compressing with optimal block size %d ===",
                   optimal_block_size);
        
        auto optimal_settings = compression_setup_.settings;
        optimal_settings.block_size = optimal_block_size;
        compressor_->updateSettings(optimal_settings);
        
        auto compress_result = compressor_->compress(input_file_);
        
        if (compress_result.success) {
            last_compression_result_ = std::move(compress_result);
            const auto& stored = *last_compression_result_;
            RCLCPP_INFO(this->get_logger(), "Compression successful!");
            RCLCPP_INFO(this->get_logger(),
                        "Original size: %zu bytes", stored.original_size);
            RCLCPP_INFO(this->get_logger(),
                        "Compressed size: %zu bytes", stored.compressed_size);
            RCLCPP_INFO(this->get_logger(),
                        "Compression ratio: %.6f", stored.compression_ratio);
            return &stored;
        } else {
            last_compression_result_.reset();
            RCLCPP_ERROR(this->get_logger(),
                        "Compression failed: %s", compress_result.error_message.c_str());
            return nullptr;
        }
    }
    
    
    std::string input_file_;
    int min_block_size_;
    int max_block_size_;
    int step_size_;
    double voxel_size_;
    bool verbose_;
    bool run_once_;
    
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr optimal_block_size_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr compression_ratio_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr optimization_service_;
    
    
    std::unique_ptr<VqOccupancyCompressor> compressor_;
    config::CompressionSetup compression_setup_;
    
    std::optional<CompressionResult> last_compression_result_;
    
    void displayCompressionSummary(const CompressionResult& compression_result,
                                   int optimal_block_size) {
        const VoxelGrid& grid = compression_result.voxel_grid;
        VoxelCoord dims = grid.getDimensions();
        size_t total_voxels = static_cast<size_t>(dims.x) * dims.y * dims.z;
        if (total_voxels == 0) {
            RCLCPP_WARN(this->get_logger(),
                        "Voxel grid is empty; skipping detailed comparison.");
            return;
        }
        size_t occupied_voxels = grid.getOccupiedVoxelCount();
        size_t one_byte_per_voxel_size = total_voxels;
        size_t one_bit_per_voxel_size = (total_voxels + 7) / 8;

        const double voxel_size = compression_result.voxel_size > 0.0f
                                      ? static_cast<double>(compression_result.voxel_size)
                                      : voxel_size_;
        double world_size_x = dims.x * voxel_size;
        double world_size_y = dims.y * voxel_size;
        double world_size_z = dims.z * voxel_size;

        double occupancy_percent =
            static_cast<double>(total_voxels) > 0.0
                ? (100.0 * static_cast<double>(occupied_voxels) /
                   static_cast<double>(total_voxels))
                : 0.0;

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
                    occupancy_percent);
        RCLCPP_INFO(this->get_logger(),
                    "1 byte/voxel map size: %zu bytes",
                    one_byte_per_voxel_size);
        RCLCPP_INFO(this->get_logger(),
                    "1 bit/voxel map size: %zu bytes",
                    one_bit_per_voxel_size);

        size_t dictionary_bytes = 0;
        for (const auto& pattern : compression_result.pattern_dictionary) {
            dictionary_bytes += pattern.size();
        }
        const size_t index_entry_bytes = (compression_result.index_bit_size + 7) / 8;
        size_t index_bytes = compression_result.block_indices.size() * index_entry_bytes;
        size_t compressed_size = compression_result.compressed_size;
        if (compressed_size == 0) {
            compressed_size = dictionary_bytes + index_bytes;
        }

        int effective_block_size = compression_result.block_size > 0
                                       ? compression_result.block_size
                                       : optimal_block_size;
        int x_blocks =
            effective_block_size > 0 ? (dims.x + effective_block_size - 1) / effective_block_size : 0;
        int y_blocks =
            effective_block_size > 0 ? (dims.y + effective_block_size - 1) / effective_block_size : 0;
        int z_blocks =
            effective_block_size > 0 ? (dims.z + effective_block_size - 1) / effective_block_size : 0;
        size_t total_blocks = static_cast<size_t>(x_blocks) * y_blocks * z_blocks;

        float compression_vs_one_byte = one_byte_per_voxel_size > 0
            ? static_cast<float>(compressed_size) / static_cast<float>(one_byte_per_voxel_size)
            : 0.0f;
        float compression_vs_one_bit = one_bit_per_voxel_size > 0
            ? static_cast<float>(compressed_size) / static_cast<float>(one_bit_per_voxel_size)
            : 0.0f;

        RCLCPP_INFO(this->get_logger(),
                    "Compressed map (block_size=%d):", effective_block_size);
        RCLCPP_INFO(this->get_logger(),
                    "  - Codebook: %zu bytes", dictionary_bytes);
        RCLCPP_INFO(this->get_logger(),
                    "  - Index map: %zu bytes", index_bytes);
        RCLCPP_INFO(this->get_logger(),
                    "  - Total: %zu bytes", compressed_size);
        if (total_blocks > 0) {
            RCLCPP_INFO(this->get_logger(),
                        "  - Blocks: %d x %d x %d = %zu",
                        x_blocks, y_blocks, z_blocks, total_blocks);
        }
        if (one_byte_per_voxel_size > 0) {
            RCLCPP_INFO(this->get_logger(),
                        "Compression ratio (vs 1 byte/voxel): %.6f (%.1fx smaller)",
                        compression_vs_one_byte,
                        compression_vs_one_byte > 0.0f ? (1.0f / compression_vs_one_byte) : 0.0f);
        }
        if (one_bit_per_voxel_size > 0) {
            RCLCPP_INFO(this->get_logger(),
                        "Compression ratio (vs 1 bit/voxel): %.6f (%.1fx smaller)",
                        compression_vs_one_bit,
                        compression_vs_one_bit > 0.0f ? (1.0f / compression_vs_one_bit) : 0.0f);
        }
        RCLCPP_INFO(this->get_logger(),
                    "========================================");
    }
};

} 

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vq_occupancy_compressor::BlockSizeOptimizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
