// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "pointcloud_compressor/config/CompressorConfig.hpp"
#include "pointcloud_compressor/config/ConfigTransforms.hpp"
#include "pointcloud_compressor/msg/pattern_dictionary.hpp"
#include "pointcloud_compressor/services/CompressionExecutor.hpp"

class PointCloudCompressorNode : public rclcpp::Node {
public:
    PointCloudCompressorNode()
        : rclcpp::Node("pointcloud_compressor_node") {
        declareParameters();
        loadParameters();

        if (!validateConfiguration()) {
            return;
        }
        RCLCPP_INFO(get_logger(),
                    "PointCloud Compressor Node initialized - Input: %s, Voxel: %.3f, Block: %d",
                    input_file_.c_str(), voxel_size_, block_size_);
        setupPublishers();
        compressAndPublish();
    }

    ~PointCloudCompressorNode() override = default;

private:
    void declareParameters() {
        this->declare_parameter("input_file", "");
        this->declare_parameter("config_file", "");
        this->declare_parameter("voxel_size", 0.01);
        this->declare_parameter("block_size", 8);
        this->declare_parameter("min_points_threshold", 1);
        this->declare_parameter("publish_occupied_voxel_markers", false);
        this->declare_parameter("save_hdf5", false);
        this->declare_parameter("hdf5_output_file", "/tmp/compressed_map.h5");
        this->declare_parameter("save_raw_hdf5", false);
        this->declare_parameter("raw_hdf5_output_file", "/tmp/raw_voxel_grid.h5");
    }

    void loadParameters() {
        input_file_ = this->get_parameter("input_file").as_string();

        voxel_size_ = this->get_parameter("voxel_size").as_double();
        block_size_ = this->get_parameter("block_size").as_int();
        min_points_threshold_ = this->get_parameter("min_points_threshold").as_int();
        publish_occupied_voxel_markers_ = this->get_parameter("publish_occupied_voxel_markers").as_bool();

        save_hdf5_ = this->get_parameter("save_hdf5").as_bool();
        hdf5_output_file_ = this->get_parameter("hdf5_output_file").as_string();
        save_raw_hdf5_ = this->get_parameter("save_raw_hdf5").as_bool();
        raw_hdf5_output_file_ = this->get_parameter("raw_hdf5_output_file").as_string();

        const std::string config_path = this->get_parameter("config_file").as_string();
        if (!config_path.empty()) {
            try {
                auto config = pointcloud_compressor::config::loadCompressorConfigFromYaml(config_path);
                if (!config.input_file.empty()) {
                    input_file_ = config.input_file;
                }
                voxel_size_ = config.voxel_size;
                block_size_ = config.block_size;
                min_points_threshold_ = config.min_points_threshold;
                publish_occupied_voxel_markers_ = config.publish_occupied_voxel_markers;
                save_hdf5_ = config.save_hdf5;
                if (!config.hdf5_output_file.empty()) {
                    hdf5_output_file_ = config.hdf5_output_file;
                }
                save_raw_hdf5_ = config.save_raw_hdf5;
                if (!config.raw_hdf5_output_file.empty()) {
                    raw_hdf5_output_file_ = config.raw_hdf5_output_file;
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Failed to load config file '%s': %s",
                             config_path.c_str(), e.what());
            }
        }
    }

    bool validateConfiguration() {
        if (input_file_.empty()) {
            RCLCPP_ERROR(get_logger(),
                         "No input file specified. Set the 'input_file' parameter.");
            return false;
        }

        if (!std::filesystem::exists(input_file_)) {
            RCLCPP_ERROR(get_logger(), "Input file does not exist: %s", input_file_.c_str());
            return false;
        }

        if (voxel_size_ <= 0.0) {
            RCLCPP_ERROR(get_logger(), "voxel_size must be positive (current: %.3f)", voxel_size_);
            return false;
        }

        if (block_size_ <= 0) {
            RCLCPP_ERROR(get_logger(), "block_size must be greater than zero (current: %d)", block_size_);
            return false;
        }

        if (min_points_threshold_ < 0) {
            RCLCPP_ERROR(get_logger(), "min_points_threshold must be non-negative (current: %d)",
                         min_points_threshold_);
            return false;
        }

        return true;
    }

    void setupPublishers() {
        pattern_dict_pub_ = this->create_publisher<pointcloud_compressor::msg::PatternDictionary>(
            "pattern_dictionary", rclcpp::QoS(rclcpp::KeepLast(10)));

        if (publish_occupied_voxel_markers_) {
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "occupied_voxel_markers", rclcpp::QoS(rclcpp::KeepLast(10)));
        }
    }

    pointcloud_compressor::config::CompressorConfig configFromParameters() const {
        pointcloud_compressor::config::CompressorConfig config;
        config.input_file = input_file_;
        config.voxel_size = voxel_size_;
        config.block_size = block_size_;
        config.min_points_threshold = min_points_threshold_;
        config.publish_occupied_voxel_markers = publish_occupied_voxel_markers_;
        config.save_hdf5 = save_hdf5_;
        config.hdf5_output_file = hdf5_output_file_;
        config.save_raw_hdf5 = save_raw_hdf5_;
        config.raw_hdf5_output_file = raw_hdf5_output_file_;
        return config;
    }

    void compressAndPublish() {
        auto setup = pointcloud_compressor::config::buildCompressionSetup(configFromParameters());
        auto errors = pointcloud_compressor::config::validateCompressionSetup(setup);
        if (!errors.empty()) {
            for (const auto& err : errors) {
                RCLCPP_ERROR(get_logger(), "%s", err.c_str());
            }
            return;
        }

        std::string error_message;
        const bool success = pointcloud_compressor::services::runCompression(
            setup,
            [this](const PCCCompressionReport& report,
                   pointcloud_compressor::io::CompressionReportBuilder&) {
                auto msg = createPatternDictionaryMessage(report);
                pattern_dict_pub_->publish(msg);

                if (publish_occupied_voxel_markers_ && marker_pub_) {
                    auto markers = createOccupiedVoxelMarkers(report);
                    if (markers.has_value()) {
                        marker_pub_->publish(markers.value());
                    } else {
                        RCLCPP_WARN(get_logger(), "Failed to create occupied voxel markers");
                    }
                }

                if (report.error_message && report.error_message[0] != '\0') {
                    RCLCPP_WARN(get_logger(), "Compression warning: %s", report.error_message);
                }
            },
            &error_message);

        if (!success) {
            RCLCPP_ERROR(get_logger(), "Compression failed: %s", error_message.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "Processing completed!!");
    }

    pointcloud_compressor::msg::PatternDictionary createPatternDictionaryMessage(
        const PCCCompressionReport& report) {
        pointcloud_compressor::msg::PatternDictionary msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";

        msg.voxel_size = voxel_size_;
        msg.block_size = static_cast<uint32_t>(block_size_);
        msg.min_points_threshold = static_cast<uint32_t>(min_points_threshold_);

        msg.voxel_grid_dimensions.x = report.grid.dimensions[0];
        msg.voxel_grid_dimensions.y = report.grid.dimensions[1];
        msg.voxel_grid_dimensions.z = report.grid.dimensions[2];

        msg.voxel_grid_origin.x = report.grid.origin[0];
        msg.voxel_grid_origin.y = report.grid.origin[1];
        msg.voxel_grid_origin.z = report.grid.origin[2];

        msg.total_blocks = report.indices.total_blocks;
        msg.blocks_x = report.grid.blocks_per_axis[0];
        msg.blocks_y = report.grid.blocks_per_axis[1];
        msg.blocks_z = report.grid.blocks_per_axis[2];
        msg.index_bit_size = report.indices.index_bit_size;

        msg.block_indices_data.clear();
        if (report.indices.data && report.indices.size > 0) {
            msg.block_indices_data.insert(
                msg.block_indices_data.end(),
                report.indices.data,
                report.indices.data + report.indices.size);
        }

        msg.num_patterns = report.dictionary.num_patterns;
        msg.pattern_size_bytes = report.dictionary.pattern_size_bytes;

        msg.dictionary_data.clear();
        if (report.dictionary.data && report.dictionary.size > 0) {
            msg.dictionary_data.insert(
                msg.dictionary_data.end(),
                report.dictionary.data,
                report.dictionary.data + report.dictionary.size);
        }

        msg.checksum = 0;
        msg.compression_ratio = static_cast<float>(report.statistics.compression_ratio);
        msg.original_point_count = report.statistics.original_point_count;
        msg.compressed_data_size = report.statistics.compressed_data_size;

        return msg;
    }

    std::optional<visualization_msgs::msg::MarkerArray> createOccupiedVoxelMarkers(
        const PCCCompressionReport& report) {
        if (!report.occupancy.occupancy || report.occupancy.size == 0) {
            return std::nullopt;
        }

        const uint32_t dim_x = report.occupancy.dimensions[0];
        const uint32_t dim_y = report.occupancy.dimensions[1];
        const uint32_t dim_z = report.occupancy.dimensions[2];

        const std::size_t expected_size =
            static_cast<std::size_t>(dim_x) *
            static_cast<std::size_t>(dim_y) *
            static_cast<std::size_t>(dim_z);

        if (expected_size != report.occupancy.size) {
            RCLCPP_ERROR(get_logger(),
                         "Occupancy buffer size mismatch: expected %zu, actual %zu",
                         expected_size, report.occupancy.size);
            return std::nullopt;
        }

        visualization_msgs::msg::MarkerArray marker_array;
        const double origin_x = static_cast<double>(report.occupancy.origin[0]);
        const double origin_y = static_cast<double>(report.occupancy.origin[1]);
        const double origin_z = static_cast<double>(report.occupancy.origin[2]);
        const double voxel_size = voxel_size_;

        const int max_points_per_marker = 10000;
        std::vector<geometry_msgs::msg::Point> current_points;
        current_points.reserve(max_points_per_marker);

        std::size_t index = 0;
        int marker_id = 0;

        auto flushMarker = [&](std::vector<geometry_msgs::msg::Point>& points) {
            if (points.empty()) {
                return;
            }

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->get_clock()->now();
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = voxel_size * 0.9;
            marker.scale.y = voxel_size * 0.9;
            marker.scale.z = voxel_size * 0.9;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5f;
            marker.points = points;

            marker_array.markers.push_back(std::move(marker));
            points.clear();
            points.reserve(max_points_per_marker);
        };

        for (uint32_t z = 0; z < dim_z; ++z) {
            for (uint32_t y = 0; y < dim_y; ++y) {
                for (uint32_t x = 0; x < dim_x; ++x, ++index) {
                    if (report.occupancy.occupancy[index] == 0) {
                        continue;
                    }

                    geometry_msgs::msg::Point point;
                    point.x = origin_x + (static_cast<double>(x) + 0.5) * voxel_size;
                    point.y = origin_y + (static_cast<double>(y) + 0.5) * voxel_size;
                    point.z = origin_z + (static_cast<double>(z) + 0.5) * voxel_size;
                    current_points.push_back(point);

                    if (static_cast<int>(current_points.size()) >= max_points_per_marker) {
                        flushMarker(current_points);
                    }
                }
            }
        }

        flushMarker(current_points);
        return marker_array;
    }

    rclcpp::Publisher<pointcloud_compressor::msg::PatternDictionary>::SharedPtr pattern_dict_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    std::string input_file_;
    double voxel_size_{0.01};
    int block_size_{8};
    int min_points_threshold_{1};
    bool publish_occupied_voxel_markers_{false};
    bool save_hdf5_{false};
    std::string hdf5_output_file_;
    bool save_raw_hdf5_{false};
    std::string raw_hdf5_output_file_;
    double bounding_box_margin_ratio_{0.0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int exit_code = 0;

    try {
        auto node = std::make_shared<PointCloudCompressorNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pointcloud_compressor_node"),
                     "Unhandled exception: %s", e.what());
        exit_code = 1;
    } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("pointcloud_compressor_node"),
                     "Unhandled unknown exception");
        exit_code = 1;
    }

    rclcpp::shutdown();
    return exit_code;
}
