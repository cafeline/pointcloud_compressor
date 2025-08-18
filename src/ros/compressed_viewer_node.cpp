#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>

#include "pointcloud_compressor/msg/pattern_dictionary.hpp"

class CompressedViewerNode : public rclcpp::Node
{
public:
    CompressedViewerNode() : Node("compressed_viewer_node"), pattern_published_(false)
    {
        // Create subscriber for pattern dictionary
        pattern_dict_sub_ = this->create_subscription<pointcloud_compressor::msg::PatternDictionary>(
            "pattern_dictionary", 10,
            std::bind(&CompressedViewerNode::patternDictionaryCallback, this, std::placeholders::_1));
        
        // Create publisher for pattern markers
        pattern_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "pattern_markers", 10);
        
        RCLCPP_INFO(this->get_logger(), "Compressed Viewer Node initialized");
        RCLCPP_INFO(this->get_logger(), "Waiting for pattern dictionary...");
    }

private:
    void patternDictionaryCallback(const pointcloud_compressor::msg::PatternDictionary::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received pattern dictionary with %u patterns", msg->num_patterns);
        RCLCPP_INFO(this->get_logger(), "Voxel size: %.3f", msg->voxel_size);
        RCLCPP_INFO(this->get_logger(), "Block size: %u", msg->block_size);
        RCLCPP_INFO(this->get_logger(), "Grid dimensions: %.1fx%.1fx%.1f", 
                    msg->voxel_grid_dimensions.x, 
                    msg->voxel_grid_dimensions.y, 
                    msg->voxel_grid_dimensions.z);
        RCLCPP_INFO(this->get_logger(), "Total blocks: %u", msg->total_blocks);
        
        // Reconstruct and publish pattern markers
        publishPatternMarkers(msg);
    }
    
    void publishPatternMarkers(const pointcloud_compressor::msg::PatternDictionary::SharedPtr msg)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // Delete all previous markers
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header = msg->header;
        delete_marker.header.frame_id = "map";
        delete_marker.ns = "patterns";
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);
        
        // Calculate voxels per block
        const int voxels_per_block = msg->block_size * msg->block_size * msg->block_size;
        const int bits_per_block = voxels_per_block;
        const int bytes_per_pattern = msg->pattern_size_bytes;
        
        // Check if we have block indices
        if (msg->block_indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "No block indices available, cannot reconstruct");
            pattern_marker_pub_->publish(marker_array);  // Still publish delete marker
            return;
        }
        
        // Debug: Check dictionary data
        RCLCPP_INFO(this->get_logger(), "Dictionary data size: %zu bytes", msg->dictionary_data.size());
        RCLCPP_INFO(this->get_logger(), "Block indices size: %zu", msg->block_indices.size());
        
        // Reconstruct blocks from patterns
        int marker_id = 0;
        const float voxel_size = msg->voxel_size;
        // const float block_size_world = msg->block_size * voxel_size;  // Not used currently
        
        // Iterate through all blocks
        for (size_t block_idx = 0; block_idx < msg->block_indices.size(); ++block_idx) {
            uint8_t pattern_idx = msg->block_indices[block_idx];
            
            // Skip empty blocks (pattern index 255 typically indicates empty)
            if (pattern_idx == 255) {
                continue;
            }
            
            // Check if pattern index is valid
            if (pattern_idx >= msg->num_patterns) {
                RCLCPP_WARN(this->get_logger(), "Invalid pattern index %u at block %zu", 
                           pattern_idx, block_idx);
                continue;
            }
            
            // Calculate block position in grid
            int bx = block_idx % msg->blocks_x;
            int by = (block_idx / msg->blocks_x) % msg->blocks_y;
            int bz = block_idx / (msg->blocks_x * msg->blocks_y);
            
            // Get pattern data
            const size_t pattern_offset = pattern_idx * bytes_per_pattern;
            if (pattern_offset + bytes_per_pattern > msg->dictionary_data.size()) {
                RCLCPP_WARN(this->get_logger(), "Pattern data out of bounds for pattern %u", 
                           pattern_idx);
                continue;
            }
            
            // Extract pattern bits
            std::vector<bool> pattern_bits(bits_per_block);
            for (int bit = 0; bit < bits_per_block; ++bit) {
                int byte_idx = bit / 8;
                int bit_idx = bit % 8;
                if (pattern_offset + byte_idx < msg->dictionary_data.size()) {
                    pattern_bits[bit] = (msg->dictionary_data[pattern_offset + byte_idx] >> bit_idx) & 1;
                }
            }
            
            // Create markers for occupied voxels in this block
            for (uint32_t vz = 0; vz < msg->block_size; ++vz) {
                for (uint32_t vy = 0; vy < msg->block_size; ++vy) {
                    for (uint32_t vx = 0; vx < msg->block_size; ++vx) {
                        int voxel_idx = vz * msg->block_size * msg->block_size + 
                                       vy * msg->block_size + vx;
                        
                        if (voxel_idx < bits_per_block && pattern_bits[voxel_idx]) {
                            // Create marker for this voxel
                            visualization_msgs::msg::Marker marker;
                            marker.header = msg->header;
                            marker.header.frame_id = "map";
                            marker.ns = "patterns";
                            marker.id = marker_id++;
                            marker.type = visualization_msgs::msg::Marker::CUBE;
                            marker.action = visualization_msgs::msg::Marker::ADD;
                            
                            // Calculate world position
                            marker.pose.position.x = msg->voxel_grid_origin.x + 
                                                    (bx * msg->block_size + vx + 0.5) * voxel_size;
                            marker.pose.position.y = msg->voxel_grid_origin.y + 
                                                    (by * msg->block_size + vy + 0.5) * voxel_size;
                            marker.pose.position.z = msg->voxel_grid_origin.z + 
                                                    (bz * msg->block_size + vz + 0.5) * voxel_size;
                            
                            // Orientation
                            marker.pose.orientation.w = 1.0;
                            
                            // Scale (slightly smaller than voxel size for visibility)
                            marker.scale.x = voxel_size * 0.9;
                            marker.scale.y = voxel_size * 0.9;
                            marker.scale.z = voxel_size * 0.9;
                            
                            // Color based on pattern index (different colors for different patterns)
                            float hue = (pattern_idx * 360.0f) / msg->num_patterns;
                            auto rgb = hsvToRgb(hue, 1.0, 1.0);
                            marker.color.r = rgb[0];
                            marker.color.g = rgb[1];
                            marker.color.b = rgb[2];
                            marker.color.a = 0.7;
                            
                            marker.lifetime = rclcpp::Duration(0, 0);  // Permanent
                            
                            marker_array.markers.push_back(marker);
                        }
                    }
                }
            }
        }
        
        // Publish the marker array
        RCLCPP_INFO(this->get_logger(), "Publishing %zu pattern markers", 
                    marker_array.markers.size() - 1);  // -1 for delete marker
        
        // Actually publish the message
        pattern_marker_pub_->publish(marker_array);
        
        // Log success
        RCLCPP_INFO(this->get_logger(), "Pattern markers published successfully");
        
        // Debug: Check if markers were actually added
        if (marker_array.markers.size() <= 1) {
            RCLCPP_WARN(this->get_logger(), "Warning: No pattern markers generated, only delete marker");
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully published %zu markers to /pattern_markers topic", 
                        marker_array.markers.size());
        }
    }
    
    std::array<float, 3> hsvToRgb(float h, float s, float v)
    {
        float c = v * s;
        float x = c * (1 - std::abs(std::fmod(h / 60.0, 2) - 1));
        float m = v - c;
        
        float r, g, b;
        if (h < 60) {
            r = c; g = x; b = 0;
        } else if (h < 120) {
            r = x; g = c; b = 0;
        } else if (h < 180) {
            r = 0; g = c; b = x;
        } else if (h < 240) {
            r = 0; g = x; b = c;
        } else if (h < 300) {
            r = x; g = 0; b = c;
        } else {
            r = c; g = 0; b = x;
        }
        
        return {r + m, g + m, b + m};
    }
    
    // Member variables
    rclcpp::Subscription<pointcloud_compressor::msg::PatternDictionary>::SharedPtr pattern_dict_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pattern_marker_pub_;
    bool pattern_published_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<CompressedViewerNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("compressed_viewer_node"), 
                     "Exception in main: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}