#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "vq_occupancy_compressor/cli/OptimizeWorkflow.hpp"

namespace fs = std::filesystem;

namespace {

std::string writeTestPointCloud(const fs::path& dir) {
  fs::create_directories(dir);
  const fs::path file = dir / "optimize_input.ply";
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

std::string writeOptimizeConfig(const fs::path& dir,
                                const std::string& pointcloud_path,
                                const std::string& output_path) {
  const fs::path config_file = dir / "optimize_config.yaml";
  std::ofstream out(config_file);
  out << "block_size_optimizer:\n";
  out << "  ros__parameters:\n";
  out << "    input_file: \"" << pointcloud_path << "\"\n";
  out << "    voxel_size: 0.1\n";
  out << "    min_block_size: 2\n";
  out << "    max_block_size: 4\n";
  out << "    step_size: 1\n";
  out << "    save_hdf5: true\n";
  out << "    hdf5_output_file: \"" << output_path << "\"\n";
  out << "    save_raw_hdf5: false\n";
  out << "    run_once: true\n";
  out.close();
  return config_file.string();
}

}  // namespace

TEST(CliOptimize, GeneratesCompressedArtifactByDefault) {
  const fs::path temp_dir = fs::temp_directory_path() / "vq_occupancy_compressor_cli_opt";
  if (fs::exists(temp_dir)) {
    fs::remove_all(temp_dir);
  }
  fs::create_directories(temp_dir);

  const std::string pointcloud_path = writeTestPointCloud(temp_dir);
  const fs::path output_file = temp_dir / "optimized_output.h5";
  const std::string config_path =
      writeOptimizeConfig(temp_dir, pointcloud_path, output_file.string());

  ASSERT_FALSE(fs::exists(output_file));

  const auto result = vq_occupancy_compressor::cli::runOptimizeWorkflow(config_path);

  EXPECT_TRUE(fs::exists(output_file));
  EXPECT_GT(fs::file_size(output_file), 0u);
  EXPECT_GT(result.block_result.optimal_block_size, 0);
  EXPECT_EQ(result.hdf5_output_file, output_file.string());

  fs::remove_all(temp_dir);
}
