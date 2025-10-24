// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <filesystem>

#include "pointcloud_compressor/runtime/CompressionReportBuilder.hpp"
#include "pointcloud_compressor/runtime/Hdf5Writers.hpp"
#include "pointcloud_compressor/runtime/RuntimeHelpers.hpp"

using pointcloud_compressor::CompressionResult;
using pointcloud_compressor::VoxelGrid;
using pointcloud_compressor::runtime::CompressionReportBuilder;
using pointcloud_compressor::runtime::writeCompressedMap;
using pointcloud_compressor::runtime::writeRawVoxelGrid;

namespace {

CompressionResult makeTestResult() {
    CompressionResult result;
    result.success = true;
    result.pattern_dictionary = {{0xAA, 0xBB}};
    result.num_unique_patterns = 1;
    result.block_indices = {0};
    result.max_index = 0;
    result.index_bit_size = 8;
    result.num_blocks = 1;
    result.point_count = 4;
    result.compressed_size = 4;
    result.block_size = 1;
    result.grid_dimensions = {1.0, 1.0, 1.0};
    result.grid_origin = {0.0, 0.0, 0.0};
    result.blocks_count = {1, 1, 1};
    result.compression_ratio = 0.5f;
    result.voxel_grid.initialize(1, 1, 1, 0.1f);
    result.voxel_grid.setOrigin(0.0f, 0.0f, 0.0f);
    result.voxel_grid.setVoxel(0, 0, 0, true);
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
    request.save_raw_hdf5 = true;
    request.raw_hdf5_output_path = "/tmp/raw.h5";
    request.bounding_box_margin_ratio = 0.0;
    return request;
}

}  // namespace

TEST(Hdf5WritersTest, SkipsWhenPathEmpty) {
    CompressionResult result = makeTestResult();
    auto request = makeRequest();

    CompressionReportBuilder builder;
    std::vector<uint8_t> dict_buf;
    std::vector<uint8_t> idx_buf;
    std::vector<uint8_t> occ_buf;
    std::string error;
    builder.build(result, request, dict_buf, idx_buf, occ_buf, error);
    auto data = builder.toCompressedMapData(result, request, dict_buf,
                                            pointcloud_compressor::runtime::convertBlockIndicesToU32(result.block_indices));

    std::string write_error;
    EXPECT_FALSE(writeCompressedMap("", data, write_error));
    EXPECT_FALSE(writeRawVoxelGrid("", result, request, occ_buf, write_error));
}

TEST(Hdf5WritersTest, WritesCompressedAndRawFiles) {
    CompressionResult result = makeTestResult();
    auto request = makeRequest();

    CompressionReportBuilder builder;
    std::vector<uint8_t> dict_buf;
    std::vector<uint8_t> idx_buf;
    std::vector<uint8_t> occ_buf;
    std::string error;
    builder.build(result, request, dict_buf, idx_buf, occ_buf, error);
    auto data = builder.toCompressedMapData(
        result, request, dict_buf,
        pointcloud_compressor::runtime::convertBlockIndicesToU32(result.block_indices));

    auto temp_dir = std::filesystem::temp_directory_path();
    auto compressed_path = (temp_dir / "pcc_writer_test.h5").string();
    auto raw_path = (temp_dir / "pcc_writer_raw.h5").string();

    std::string write_error;
    EXPECT_TRUE(writeCompressedMap(compressed_path, data, write_error));
    EXPECT_TRUE(std::filesystem::exists(compressed_path));

    EXPECT_TRUE(writeRawVoxelGrid(raw_path, result, request, occ_buf, write_error));
    EXPECT_TRUE(std::filesystem::exists(raw_path));

    std::filesystem::remove(compressed_path);
    std::filesystem::remove(raw_path);
}
