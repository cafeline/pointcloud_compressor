#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <vector>

#include "pointcloud_compressor/core/PointCloudCompressor.hpp"

namespace fs = std::filesystem;

namespace {

std::string writeTestPointCloud(const fs::path& dir) {
  fs::create_directories(dir);
  const fs::path file = dir / "test_cloud.ply";
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
  out.close();
  return file.string();
}

std::vector<fs::path> listDirectoryFiles(const fs::path& dir) {
  std::vector<fs::path> files;
  for (const auto& entry : fs::directory_iterator(dir)) {
    files.push_back(entry.path());
  }
  return files;
}

}  // namespace

TEST(EndToEnd, CompressInMemoryWithoutTempFiles) {
  const fs::path temp_dir = fs::temp_directory_path() / fs::path("pointcloud_compressor_e2e");
  if (fs::exists(temp_dir)) {
    fs::remove_all(temp_dir);
  }
  fs::create_directories(temp_dir);

  const std::string ply_file = writeTestPointCloud(temp_dir);

  pointcloud_compressor::CompressionSettings settings;
  settings.voxel_size = 0.1f;
  settings.block_size = 2;
  pointcloud_compressor::PointCloudCompressor compressor(settings);

  auto result = compressor.compress(ply_file);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.error_message, "");
  EXPECT_GT(result.point_count, 0u);
  EXPECT_FALSE(result.pattern_dictionary.empty());
  EXPECT_FALSE(result.block_indices.empty());

  auto files_after = listDirectoryFiles(temp_dir);
  ASSERT_EQ(files_after.size(), 1u);
  EXPECT_EQ(files_after.front().filename(), "test_cloud.ply");

  fs::remove_all(temp_dir);
}

