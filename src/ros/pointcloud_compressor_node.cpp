#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <filesystem>
#include <chrono>
#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>

#include "pointcloud_compressor/msg/pattern_dictionary.hpp"
#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/core/VoxelProcessor.hpp"
#include "pointcloud_compressor/io/PointCloudIO.hpp"
#include "pointcloud_compressor/io/HDF5IO.hpp"

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
        this->declare_parameter("save_hdf5", false);
        this->declare_parameter("hdf5_output_file", "/tmp/compressed_map.h5");
        // Save raw (pre-compression) occupancy grid
        this->declare_parameter("save_raw_hdf5", false);
        this->declare_parameter("raw_hdf5_output_file", "/tmp/raw_voxel_grid.h5");
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

        // Get HDF5 settings
        save_hdf5_ = this->get_parameter("save_hdf5").as_bool();
        hdf5_output_file_ = this->get_parameter("hdf5_output_file").as_string();
        // Raw occupancy grid save parameters
        save_raw_hdf5_ = this->get_parameter("save_raw_hdf5").as_bool();
        raw_hdf5_output_file_ = this->get_parameter("raw_hdf5_output_file").as_string();
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

        // Save to HDF5 if requested
        if (save_hdf5_) {
            saveToHDF5(compression_result);
        }
        // Save raw occupancy grid if requested
        if (save_raw_hdf5_) {
            saveRawVoxelGridToHDF5(compression_result);
        }

        // Publish results
        publishPatternDictionary(compression_result);
        publishOccupiedVoxelMarkers();

        compressed_published_ = true;
    }

    pointcloud_compressor::CompressionResult performCompression()
    {
        std::string temp_prefix = "/tmp/compressed_" + std::to_string(this->now().nanoseconds());

        auto compression_result = compressor_->compress(input_file_, temp_prefix);

        if (compression_result.success) {
            // Store max index for reuse
            compression_result.max_index = 0;
            if (!compression_result.block_indices.empty()) {
                compression_result.max_index = *std::max_element(compression_result.block_indices.begin(),
                                                                compression_result.block_indices.end());
            }
            logCompressionSummary(compression_result);

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

        // Use pre-calculated index bit size from compression result
        msg.index_bit_size = result.index_bit_size;

        // Pack indices based on bit size
        msg.block_indices_data.clear();

        if (msg.index_bit_size == 8) {
            // Pack as 8-bit indices
            msg.block_indices_data.reserve(result.block_indices.size());
            for (uint64_t idx : result.block_indices) {
                msg.block_indices_data.push_back(static_cast<uint8_t>(idx));
            }
        } else if (msg.index_bit_size == 16) {
            // Pack as 16-bit indices (little-endian)
            msg.block_indices_data.reserve(result.block_indices.size() * 2);
            for (uint64_t idx : result.block_indices) {
                msg.block_indices_data.push_back(static_cast<uint8_t>(idx & 0xFF));         // Low byte
                msg.block_indices_data.push_back(static_cast<uint8_t>((idx >> 8) & 0xFF)); // High byte
            }
        } else if (msg.index_bit_size == 32) {
            // Pack as 32-bit indices (little-endian)
            msg.block_indices_data.reserve(result.block_indices.size() * 4);
            for (uint64_t idx : result.block_indices) {
                msg.block_indices_data.push_back(static_cast<uint8_t>(idx & 0xFF));         // Byte 0
                msg.block_indices_data.push_back(static_cast<uint8_t>((idx >> 8) & 0xFF)); // Byte 1
                msg.block_indices_data.push_back(static_cast<uint8_t>((idx >> 16) & 0xFF)); // Byte 2
                msg.block_indices_data.push_back(static_cast<uint8_t>((idx >> 24) & 0xFF)); // Byte 3
            }
        } else if (msg.index_bit_size == 64) {
            // Pack as 64-bit indices (little-endian)
            msg.block_indices_data.reserve(result.block_indices.size() * 8);
            for (uint64_t idx : result.block_indices) {
                msg.block_indices_data.push_back(static_cast<uint8_t>(idx & 0xFF));         // Byte 0
                msg.block_indices_data.push_back(static_cast<uint8_t>((idx >> 8) & 0xFF)); // Byte 1
                msg.block_indices_data.push_back(static_cast<uint8_t>((idx >> 16) & 0xFF)); // Byte 2
                msg.block_indices_data.push_back(static_cast<uint8_t>((idx >> 24) & 0xFF)); // Byte 3
                msg.block_indices_data.push_back(static_cast<uint8_t>((idx >> 32) & 0xFF)); // Byte 4
                msg.block_indices_data.push_back(static_cast<uint8_t>((idx >> 40) & 0xFF)); // Byte 5
                msg.block_indices_data.push_back(static_cast<uint8_t>((idx >> 48) & 0xFF)); // Byte 6
                msg.block_indices_data.push_back(static_cast<uint8_t>((idx >> 56) & 0xFF)); // Byte 7
            }
        }

        RCLCPP_INFO(this->get_logger(), "Using %u-bit index encoding (max index: %lu)",
                    msg.index_bit_size, result.max_index);

        // Set pattern dictionary
        msg.num_patterns = static_cast<uint32_t>(result.num_unique_patterns);
        // block_size^3 bits => ceil(bits/8) bytes
        const uint32_t pattern_bits = static_cast<uint32_t>(settings_.block_size) *
                                      static_cast<uint32_t>(settings_.block_size) *
                                      static_cast<uint32_t>(settings_.block_size);
        msg.pattern_size_bytes = (pattern_bits + 7) / 8;

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
            RCLCPP_INFO(this->get_logger(), "Skipping occupied voxel markers publication (disabled or no publisher)");
            return;
        }

        auto marker_array = createOccupiedVoxelMarkers();
        if (!marker_array.has_value()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create occupied voxel markers");
            return;
        }

        marker_pub_->publish(marker_array.value());
        RCLCPP_INFO(this->get_logger(), "Published occupied_voxel_markers: %zu markers", marker_array.value().markers.size());
    }

    std::optional<visualization_msgs::msg::MarkerArray> createOccupiedVoxelMarkers()
    {
        // Try to use cached voxel grid first
        auto cached_grid = compressor_->getCachedVoxelGrid();

        pointcloud_compressor::VoxelGrid grid;

        if (cached_grid.has_value()) {
            // Use cached grid - no need to reload and reprocess
            RCLCPP_DEBUG(this->get_logger(), "Using cached voxel grid for marker visualization");
            grid = cached_grid.value();
        } else {
            // No cached grid available - this should not happen in normal flow
            RCLCPP_ERROR(this->get_logger(), "No cached voxel grid found for marker visualization");
            return std::nullopt;
        }

        // Create marker array with CUBE_LIST for efficiency
        visualization_msgs::msg::MarkerArray marker_array;
        auto dimensions = grid.getDimensions();

        float origin_x, origin_y, origin_z;
        grid.getOrigin(origin_x, origin_y, origin_z);

        // Create CUBE_LIST markers (split into chunks for better performance)
        const int MAX_POINTS_PER_MARKER = 10000;
        std::vector<geometry_msgs::msg::Point> current_points;
        current_points.reserve(MAX_POINTS_PER_MARKER);
        int marker_id = 0;

        for (int z = 0; z < dimensions.z; ++z) {
            for (int y = 0; y < dimensions.y; ++y) {
                for (int x = 0; x < dimensions.x; ++x) {
                    if (grid.getVoxel(x, y, z)) {
                        geometry_msgs::msg::Point point;
                        point.x = origin_x + (x + 0.5) * settings_.voxel_size;
                        point.y = origin_y + (y + 0.5) * settings_.voxel_size;
                        point.z = origin_z + (z + 0.5) * settings_.voxel_size;
                        current_points.push_back(point);

                        // Create new CUBE_LIST marker when reaching limit
                        if (current_points.size() >= MAX_POINTS_PER_MARKER) {
                            marker_array.markers.push_back(createCubeListMarker(
                                marker_id++, current_points
                            ));
                            current_points.clear();
                            current_points.reserve(MAX_POINTS_PER_MARKER);
                        }
                    }
                }
            }
        }

        // Add remaining points
        if (!current_points.empty()) {
            marker_array.markers.push_back(createCubeListMarker(
                marker_id++, current_points
            ));
        }

        RCLCPP_INFO(this->get_logger(), "Created %zu CUBE_LIST markers for occupied voxels",
                    marker_array.markers.size());

        return marker_array;
    }

    visualization_msgs::msg::Marker createCubeListMarker(
        int id, const std::vector<geometry_msgs::msg::Point>& points)
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // For CUBE_LIST, pose is identity (points contain absolute positions)
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
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

        // Add all points
        marker.points = points;

        return marker;
    }

    // Removed unused function calculateAndLogPointCloudStatistics

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

        // 4. Block indices array size（実際のbit幅に基づく）
        size_t block_indices_size = result.block_indices.size() * (result.index_bit_size / 8);

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
        double occupancy_percent = occupancy_ratio * 100.0;
        double memory_reduction = (voxel_grid_size == 0)
            ? 0.0
            : (1.0 - static_cast<double>(total_compressed_size) / static_cast<double>(voxel_grid_size)) * 100.0;

        std::ostringstream summary;
        summary << "\n[ROS][Occupancy Grid]\n"
                << "  Grid dims        : " << voxel_grid_x << " x " << voxel_grid_y << " x "
                << voxel_grid_z << " (" << total_voxels << " voxels)" << '\n'
                << "  Voxel size       : " << formatFloat(settings_.voxel_size, 3) << " m" << '\n'
                << "  Occupied voxels  : " << occupied_voxels << " (" << std::fixed << std::setprecision(2)
                << occupancy_percent << "%)" << '\n'
                << "  Raw grid         : " << voxel_grid_size << " bytes ("
                << formatKilobytes(voxel_grid_size) << " KB)" << '\n'
                << "  Block size       : " << settings_.block_size << "^3 = " << block_size_3d << " voxels" << '\n'
                << "  Blocks total     : " << total_blocks << '\n'
                << "  Block indices    : " << block_indices_size << " bytes ("
                << result.index_bit_size << "-bit)" << '\n'
                << "  Pattern dict     : " << result.num_unique_patterns << " patterns, "
                << pattern_dict_size << " bytes (" << formatKilobytes(pattern_dict_size) << " KB)" << '\n'
                << "  Metadata         : " << metadata_size << " bytes" << '\n'
                << "  Compressed total : " << total_compressed_size << " bytes ("
                << formatKilobytes(total_compressed_size) << " KB)" << '\n'
                << "  Memory reduction : " << std::fixed << std::setprecision(2)
                << memory_reduction << "%%";

        RCLCPP_INFO(this->get_logger(), "%s", summary.str().c_str());
    }

    void saveToHDF5(const pointcloud_compressor::CompressionResult& result)
    {
        pointcloud_compressor::HDF5IO hdf5_io;
        pointcloud_compressor::CompressedMapData hdf5_data;

        // Set metadata
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::gmtime(&time_t), "%Y-%m-%dT%H:%M:%SZ");
        hdf5_data.creation_time = ss.str();
        hdf5_data.frame_id = "map";

        // Set compression parameters（pattern_bitsはパターンのビット数としてblock_size^3）
        hdf5_data.voxel_size = settings_.voxel_size;
        hdf5_data.dictionary_size = result.num_unique_patterns;
        hdf5_data.block_size = settings_.block_size;
        hdf5_data.pattern_bits = static_cast<uint32_t>(settings_.block_size) *
                                 static_cast<uint32_t>(settings_.block_size) *
                                 static_cast<uint32_t>(settings_.block_size);

        // Grid origin from voxel grid
        {
            float ox = 0.0f, oy = 0.0f, oz = 0.0f;
            result.voxel_grid.getOrigin(ox, oy, oz);
            hdf5_data.grid_origin = {ox, oy, oz};
        }

        // Set dictionary data
        hdf5_data.pattern_length = settings_.block_size * settings_.block_size * settings_.block_size;

        // Convert pattern dictionary to byte array
        for (const auto& pattern : result.pattern_dictionary) {
            hdf5_data.dictionary_patterns.insert(hdf5_data.dictionary_patterns.end(),
                                                pattern.begin(), pattern.end());
        }

        // Prepare dense block index grid metadata
        auto grid_dims = result.voxel_grid.getDimensions();
        const int blocks_per_dim_x = (grid_dims.x + settings_.block_size - 1) / settings_.block_size;
        const int blocks_per_dim_y = (grid_dims.y + settings_.block_size - 1) / settings_.block_size;
        const int blocks_per_dim_z = (grid_dims.z + settings_.block_size - 1) / settings_.block_size;

        hdf5_data.block_dims = {blocks_per_dim_x, blocks_per_dim_y, blocks_per_dim_z};
        hdf5_data.block_offset = {0, 0, 0};

        const size_t total_blocks = static_cast<size_t>(blocks_per_dim_x) *
                                    static_cast<size_t>(blocks_per_dim_y) *
                                    static_cast<size_t>(blocks_per_dim_z);

        auto determine_bit_width = [](uint64_t value) {
            if (value <= std::numeric_limits<uint8_t>::max()) {
                return static_cast<uint8_t>(8);
            }
            if (value <= std::numeric_limits<uint16_t>::max()) {
                return static_cast<uint8_t>(16);
            }
            if (value <= std::numeric_limits<uint32_t>::max()) {
                return static_cast<uint8_t>(32);
            }
            return static_cast<uint8_t>(64);
        };

        auto sentinel_for_width = [](uint8_t bit_width) {
            switch (bit_width) {
                case 8:  return static_cast<uint64_t>(std::numeric_limits<uint8_t>::max());
                case 16: return static_cast<uint64_t>(std::numeric_limits<uint16_t>::max());
                case 32: return static_cast<uint64_t>(std::numeric_limits<uint32_t>::max());
                default: return std::numeric_limits<uint64_t>::max();
            }
        };

        uint8_t bit_width = result.index_bit_size > 0
                                ? static_cast<uint8_t>(result.index_bit_size)
                                : determine_bit_width(result.max_index);
        if (bit_width != 8 && bit_width != 16 && bit_width != 32 && bit_width != 64) {
            bit_width = determine_bit_width(result.max_index);
        }
        bit_width = std::max(bit_width, determine_bit_width(result.max_index));
        const uint64_t sentinel = sentinel_for_width(bit_width);

        hdf5_data.block_index_bit_width = bit_width;
        hdf5_data.block_index_sentinel = sentinel;
        hdf5_data.block_indices.assign(total_blocks, sentinel);

        if (result.block_indices.size() != total_blocks) {
            RCLCPP_WARN(this->get_logger(),
                        "Block index count (%zu) does not match expected dense grid size (%zu)",
                        result.block_indices.size(), total_blocks);
        }

        const size_t copy_count = std::min(total_blocks, result.block_indices.size());
        for (size_t i = 0; i < copy_count; ++i) {
            hdf5_data.block_indices[i] = result.block_indices[i];
        }

        // Set statistics（original_points は点数を格納）
        hdf5_data.original_points = static_cast<uint64_t>(result.original_size / (3 * sizeof(float)));
        hdf5_data.compressed_voxels = result.num_blocks;
        hdf5_data.compression_ratio = result.compression_ratio;
        // Calculate bounding box from block grid metadata
        if (total_blocks > 0) {
            const double block_extent = static_cast<double>(settings_.block_size) * settings_.voxel_size;
            const auto& offset = hdf5_data.block_offset;
            const auto dims_arr = hdf5_data.block_dims;
            const std::array<double, 3> origin_d = {
                static_cast<double>(hdf5_data.grid_origin[0]),
                static_cast<double>(hdf5_data.grid_origin[1]),
                static_cast<double>(hdf5_data.grid_origin[2])
            };
            const std::array<int32_t, 3> max_offset = {
                offset[0] + dims_arr[0],
                offset[1] + dims_arr[1],
                offset[2] + dims_arr[2]
            };

            hdf5_data.bounding_box_min = {
                origin_d[0] + block_extent * static_cast<double>(offset[0]),
                origin_d[1] + block_extent * static_cast<double>(offset[1]),
                origin_d[2] + block_extent * static_cast<double>(offset[2])
            };
            hdf5_data.bounding_box_max = {
                origin_d[0] + block_extent * static_cast<double>(max_offset[0]),
                origin_d[1] + block_extent * static_cast<double>(max_offset[1]),
                origin_d[2] + block_extent * static_cast<double>(max_offset[2])
            };
        }
          
        // Write to HDF5 file
        if (hdf5_io.write(hdf5_output_file_, hdf5_data)) {
            size_t file_size = 0;
            if (std::filesystem::exists(hdf5_output_file_)) {
                file_size = std::filesystem::file_size(hdf5_output_file_);
            }

            const auto& offset = hdf5_data.block_offset;
            const auto& dims_arr = hdf5_data.block_dims;
            const auto& bbox_min = hdf5_data.bounding_box_min;
            const auto& bbox_max = hdf5_data.bounding_box_max;

            std::ostringstream summary;
            summary << "\n[ROS][HDF5 Compressed]\n"
                    << "  Path            : " << hdf5_output_file_ << '\n'
                    << "  File size       : " << file_size << " bytes (" << formatKilobytes(file_size) << " KB)" << '\n'
                    << "  Index bit width : " << static_cast<int>(hdf5_data.block_index_bit_width) << "-bit" << '\n'
                    << "  Block offset    : " << formatVector3(static_cast<double>(offset[0]),
                                                                   static_cast<double>(offset[1]),
                                                                   static_cast<double>(offset[2]), 0) << '\n'
                    << "  Block dims      : " << formatVector3(static_cast<double>(dims_arr[0]),
                                                                   static_cast<double>(dims_arr[1]),
                                                                   static_cast<double>(dims_arr[2]), 0) << '\n'
                    << "  Bounding box    : min=" << formatVector3(bbox_min[0], bbox_min[1], bbox_min[2])
                    << ", max=" << formatVector3(bbox_max[0], bbox_max[1], bbox_max[2]) << '\n'
                    << "  Patterns        : " << result.num_unique_patterns << '\n'
                    << "  Blocks          : " << result.num_blocks;

            RCLCPP_INFO(this->get_logger(), "%s", summary.str().c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save HDF5 file: %s", hdf5_io.getLastError().c_str());
        }
    }

    void saveRawVoxelGridToHDF5(const pointcloud_compressor::CompressionResult& result)
    {
        pointcloud_compressor::HDF5IO hdf5_io;
        pointcloud_compressor::HDF5IO::RawVoxelGridData raw;

        // Dimensions and voxel size
        auto dims = result.voxel_grid.getDimensions();
        raw.dim_x = static_cast<uint32_t>(dims.x);
        raw.dim_y = static_cast<uint32_t>(dims.y);
        raw.dim_z = static_cast<uint32_t>(dims.z);
        raw.voxel_size = settings_.voxel_size;

        // Origin
        float ox, oy, oz;
        result.voxel_grid.getOrigin(ox, oy, oz);
        raw.origin = {ox, oy, oz};

        // Collect occupied voxel indices
        int estimated = result.voxel_grid.getOccupiedVoxelCount();
        if (estimated > 0) raw.occupied_voxels.reserve(static_cast<size_t>(estimated));
        for (int z = 0; z < dims.z; ++z) {
            for (int y = 0; y < dims.y; ++y) {
                for (int x = 0; x < dims.x; ++x) {
                    if (result.voxel_grid.getVoxel(x, y, z)) {
                        raw.occupied_voxels.push_back({x, y, z});
                    }
                }
            }
        }

        if (hdf5_io.writeRawVoxelGrid(raw_hdf5_output_file_, raw)) {
            size_t file_size = 0;
            if (std::filesystem::exists(raw_hdf5_output_file_)) {
                file_size = std::filesystem::file_size(raw_hdf5_output_file_);
            }

            std::ostringstream summary;
            summary << "\n[ROS][HDF5 Raw]\n"
                    << "  Path            : " << raw_hdf5_output_file_ << '\n'
                    << "  File size       : " << file_size << " bytes (" << formatKilobytes(file_size) << " KB)" << '\n'
                    << "  Dimensions      : " << raw.dim_x << " x " << raw.dim_y << " x " << raw.dim_z << '\n'
                    << "  Voxel size      : " << formatFloat(raw.voxel_size, 3) << " m" << '\n'
                    << "  Origin          : " << formatVector3(raw.origin[0], raw.origin[1], raw.origin[2]) << '\n'
                    << "  Occupied voxels : " << raw.occupied_voxels.size();

            RCLCPP_INFO(this->get_logger(), "%s", summary.str().c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save raw HDF5 file: %s", hdf5_io.getLastError().c_str());
        }
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

    static std::string formatMilliseconds(double value)
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3) << value;
        return ss.str();
    }

    static std::string formatRatio(double value)
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(6) << value;
        return ss.str();
    }

    static std::string formatKilobytes(size_t bytes)
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2) << (static_cast<double>(bytes) / 1024.0);
        return ss.str();
    }

    static std::string formatFloat(double value, int precision = 3)
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(precision) << value;
        return ss.str();
    }

    static std::string formatVector3(double x, double y, double z, int precision = 3)
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(precision)
           << "(" << x << ", " << y << ", " << z << ")";
        return ss.str();
    }

    void logCompressionSummary(const pointcloud_compressor::CompressionResult& result)
    {
        const bool using_8bit = result.index_bit_size <= 8;
        std::ostringstream summary;
        summary << "\n[ROS][Compression Summary]\n"
                << "  Points            : " << result.point_count << '\n'
                << "  Blocks            : " << result.num_blocks << " ("
                << result.blocks_count.x << " x " << result.blocks_count.y << " x "
                << result.blocks_count.z << ")" << '\n'
                << "  Timings [ms]      : load=" << formatMilliseconds(result.timings.load_ms)
                << ", voxel=" << formatMilliseconds(result.timings.voxelize_ms)
                << ", dict=" << formatMilliseconds(result.timings.dictionary_ms)
                << ", save=" << formatMilliseconds(result.timings.save_ms)
                << ", total=" << formatMilliseconds(result.timings.total_ms) << '\n'
                << "  Patterns          : " << result.num_unique_patterns << " ("
                << (using_8bit ? "8-bit" : (std::to_string(result.index_bit_size) + "-bit"))
                << ")" << '\n'
                << "  Compression ratio : " << formatRatio(result.compression_ratio);

        RCLCPP_INFO(this->get_logger(), "%s", summary.str().c_str());
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
    bool save_hdf5_;
    std::string hdf5_output_file_;
    bool save_raw_hdf5_;
    std::string raw_hdf5_output_file_;
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
