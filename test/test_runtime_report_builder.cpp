// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "pointcloud_compressor/runtime/CompressionReportBuilder.hpp"
#include "pointcloud_compressor/runtime/RuntimeAPI.hpp"
#include "pointcloud_compressor/runtime/RuntimeHelpers.hpp"

using pointcloud_compressor::CompressionResult;
using pointcloud_compressor::VoxelGrid;
using pointcloud_compressor::runtime::CompressionReportBuilder;

namespace {

CompressionResult makeSampleResult() {
    CompressionResult result;
    result.success = true;
    result.pattern_dictionary = {{0xAA, 0xBB}, {0xCC}};
    result.num_unique_patterns = 2;
    result.block_indices = {0, 1};
    result.max_index = 1;
    result.index_bit_size = 8;
    result.num_blocks = 2;
    result.point_count = 10;
    result.compressed_size = 42;
    result.grid_dimensions = {1.0, 1.0, 1.0};
    result.grid_origin = {0.0, 0.0, 0.0};
    result.blocks_count = {1, 1, 2};
    result.compression_ratio = 0.5f;

    result.voxel_grid.initialize(1, 1, 2, 0.1f);
    result.voxel_grid.setOrigin(0.0f, 0.0f, 0.0f);
    result.voxel_grid.setVoxel(0, 0, 0, true);
    result.voxel_grid.setVoxel(0, 0, 1, false);

    return result;
}

PCCCompressionRequest makeRequest() {
    PCCCompressionRequest request{};
    request.input_file = "/tmp/input.pcd";
    request.voxel_size = 0.1;
    request.block_size = 1;
    request.use_8bit_indices = true;
    request.min_points_threshold = 1;
    request.save_hdf5 = true;
    request.hdf5_output_path = "/tmp/out.h5";
    request.save_raw_hdf5 = false;
    request.raw_hdf5_output_path = nullptr;
    request.bounding_box_margin_ratio = 0.0;
    return request;
}

}  // namespace

TEST(CompressionReportBuilderTest, PopulatesReportFields) {
    auto result = makeSampleResult();
    auto request = makeRequest();

    std::vector<uint8_t> dictionary_buffer;
    std::vector<uint8_t> indices_buffer;
    std::vector<uint8_t> occupancy_buffer;
    std::string error;

    CompressionReportBuilder builder;
    auto report = builder.build(result, request, dictionary_buffer, indices_buffer, occupancy_buffer, error);

    EXPECT_TRUE(report.success);
    EXPECT_EQ(2u, report.dictionary.num_patterns);
    EXPECT_EQ(dictionary_buffer.size(), report.dictionary.size);
    EXPECT_EQ(indices_buffer.size(), report.indices.size);
    EXPECT_EQ(occupancy_buffer.size(), report.occupancy.size);
    EXPECT_FLOAT_EQ(result.compression_ratio, static_cast<float>(report.statistics.compression_ratio));
    EXPECT_EQ(result.point_count, report.statistics.original_point_count);
    EXPECT_EQ(result.max_index, report.max_index);
    EXPECT_TRUE(error.empty());

    auto map_from_report = builder.toCompressedMapData(report, request.voxel_size, request.block_size);
    EXPECT_EQ(report.dictionary.num_patterns, map_from_report.dictionary_size);
    auto occupancy_from_report = builder.extractOccupancy(report);
    EXPECT_EQ(report.occupancy.size, occupancy_from_report.size());
}
