// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "pointcloud_compressor/runtime/RuntimeAPI.hpp"

namespace {

constexpr const char* kTestPointCloud = TEST_DATA_DIR "/cube.ply";

PCCCompressionRequest makeBasicRequest() {
    PCCCompressionRequest request{};
    request.input_file = kTestPointCloud;
    request.voxel_size = 0.05;
    request.block_size = 8;
    request.use_8bit_indices = false;
    request.min_points_threshold = 1;
    request.save_hdf5 = false;
    request.hdf5_output_path = nullptr;
    request.save_raw_hdf5 = false;
    request.raw_hdf5_output_path = nullptr;
    request.bounding_box_margin_ratio = 0.0;
    return request;
}

}  // namespace

TEST(PointCloudCompressorRuntimeTest, CompressesPointCloudSuccessfully) {
    auto* handle = pcc_runtime_create();
    ASSERT_NE(nullptr, handle);

    auto request = makeBasicRequest();
    auto report = pcc_runtime_compress(handle, &request);

    ASSERT_TRUE(report.success) << (report.error_message ? report.error_message : "");
    EXPECT_GT(report.dictionary.num_patterns, 0u);
    EXPECT_GT(report.dictionary.size, 0u);
    EXPECT_GT(report.indices.size, 0u);
    EXPECT_GE(report.indices.index_bit_size, 8);
    EXPECT_GT(report.grid.dimensions[0], 0.0);
    EXPECT_GT(report.statistics.original_point_count, 0u);
    EXPECT_NE(report.occupancy.occupancy, nullptr);
    EXPECT_GT(report.occupancy.size, 0u);

    const auto expected_occupancy_size =
        static_cast<std::size_t>(report.occupancy.dimensions[0]) *
        static_cast<std::size_t>(report.occupancy.dimensions[1]) *
        static_cast<std::size_t>(report.occupancy.dimensions[2]);
    EXPECT_EQ(report.occupancy.size, expected_occupancy_size);

    pcc_runtime_release_report(handle, &report);
    pcc_runtime_destroy(handle);
}
