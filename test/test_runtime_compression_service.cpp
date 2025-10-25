#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "pointcloud_compressor/bridge/RuntimeAPI.hpp"
#include "pointcloud_compressor/services/RuntimeCompressionService.hpp"

namespace fs = std::filesystem;

namespace {

fs::path writeTestPointCloud(const fs::path& dir) {
  fs::create_directories(dir);
  const fs::path file = dir / "service_test_cloud.ply";
  std::ofstream out(file);
  out << "ply\n";
  out << "format ascii 1.0\n";
  out << "element vertex 4\n";
  out << "property float x\n";
  out << "property float y\n";
  out << "property float z\n";
  out << "end_header\n";
  out << "0 0 0\n";
  out << "1 0 0\n";
  out << "0 1 0\n";
  out << "0 0 1\n";
  return file;
}

}  // namespace

TEST(RuntimeCompressionService, CompressesRequestAndProvidesReport) {
  const fs::path temp_dir = fs::temp_directory_path() / "pointcloud_compressor_service_test";
  if (fs::exists(temp_dir)) {
    fs::remove_all(temp_dir);
  }
  fs::create_directories(temp_dir);
  const fs::path ply_file = writeTestPointCloud(temp_dir);

  pointcloud_compressor::services::RuntimeCompressionService service;

  PCCCompressionRequest request{};
  std::string ply_string = ply_file.string();
  request.input_file = ply_string.c_str();
  request.voxel_size = 0.1;
  request.block_size = 2;
  request.use_8bit_indices = false;
  request.min_points_threshold = 1;
  request.save_hdf5 = false;
  request.hdf5_output_path = nullptr;
  request.save_raw_hdf5 = false;
  request.raw_hdf5_output_path = nullptr;
  request.bounding_box_margin_ratio = 0.0;

  auto report = service.compress(request);

  EXPECT_TRUE(report.success);
  EXPECT_EQ(report.error_message, nullptr);
  EXPECT_GT(report.indices.total_blocks, 0u);
  EXPECT_GT(report.dictionary.num_patterns, 0u);

  service.release(report);

  fs::remove_all(temp_dir);
}

